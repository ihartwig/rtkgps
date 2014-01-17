/**
 * Bell 202 Style Serial Modem
 * MARK: 2200 Hz
 * SPACE: 1200 Hz
 * Protocol: Similar to UART. Idles at MARK. 8E1 encoding.
 *
 * Pin 28 (D:0): Serial buffer overflow
 * Pin 27 (D:1): Interrupt debug signal
 * Pin 26 (D:2): mode select (5V receive)
 * Pin 2: ADC reference. give (3.3/2)V
 * Pin 4: DAC output
 * Pin 5: ADC input
 * Pin 7: PDI Data (Prog 1)
 * Pin 8: PDI Reset (Prog 5) with pull up
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "sinetable.h"

#define TICKS_1200HZ       26368 // theoretically 26667
#define TICKS_2200HZ       14400 // theoretically 14545
#define TICKS_9600HZ        3333

#define TICKS_2200HZ_BY_32   450 // theoretically 454
#define TICKS_1200HZ_BY_32   824 // theoretically 833

#define DATA_BUF_LEN        256
#define SEND_BYTE_BITS_LEN   10

typedef enum TONE_enum {
  TONE_UNKNOWN,
  TONE_1200HZ,
  TONE_2200HZ,
} TONE_t;

typedef enum MODE_enum {
  MODE_RECEIVE,
  MODE_SEND,
} MODE_t;

typedef enum SEND_MODE_enum {
  SEND_MODE_COLD, // we need to build a new bit before starting
  SEND_MODE_ACTIVE,
} SEND_MODE_t;

static volatile uint8_t echo_char = 42;
static volatile int cur_recv_signal = 0; // 12 bit value from the adc

extern const uint16_t sine_table[];
static volatile size_t sine_table_i = 0;
static const size_t sine_table_size = sizeof(sine_table)/sizeof(uint16_t);

static volatile TONE_t cur_send_tone = TONE_2200HZ;
static volatile TONE_t cur_recv_tone = TONE_UNKNOWN;
static volatile MODE_t cur_mode = MODE_SEND;
static volatile SEND_MODE_t cur_send_mode = SEND_MODE_COLD;
static volatile uint16_t next_bit_switch = 0;

static const uint16_t DAC_table[] = {0x0000, 0x0800, 0x0800, 0x0000};
static volatile size_t DAC_table_i = 0;

static uint8_t test_data[256];
static volatile size_t test_data_size = 0;


/**
 * 1KB Ring Buffer
 * This buffer should only be edited when in an interrupt or interrupts are
 * disabled during normal execution.
 * Empty when next == (first+1). Full when next == first
 */
typedef struct data_buf_struct {
  volatile uint8_t buf[DATA_BUF_LEN];
  volatile size_t first;
  volatile size_t next;
} data_buf_t;

data_buf_t radio_send_buf;
data_buf_t radio_recv_buf;

inline void buf_init(data_buf_t *buf) {
  buf->first = 0;
  buf->next = 1;
}

inline int buf_is_empty(data_buf_t *buf) {
  return (buf->first + 1) % DATA_BUF_LEN == buf->next % DATA_BUF_LEN;
}

inline int buf_is_full(data_buf_t *buf) {
  return buf->next % DATA_BUF_LEN == buf->first % DATA_BUF_LEN;
}

inline void buf_add(data_buf_t *buf, uint8_t data) {
  buf->buf[buf->next] = data;
  buf->next = (buf->next + 1) % DATA_BUF_LEN;
}

inline uint8_t buf_remove(data_buf_t *buf) {
  // failure case: make sure we don't corrupt the buf pointers
  // if(buf_is_empty(buf))
  uint8_t temp_data = buf->buf[buf->first];
  buf->first = (buf->first + 1) % DATA_BUF_LEN;
  return temp_data;
}

inline size_t buf_len(data_buf_t *buf) {
  if(buf->next <= buf->first) {
    // we need to pretend the buf is actually longer
    return (buf->next + DATA_BUF_LEN - 1) - buf->first;
  } else {
    return (buf->next - 1) - buf->first;
  }
}


/**
 * Current Byte Pattern Control
 * Tracks the sending of the current byte.
 */
typedef struct send_byte_struct {
  TONE_t bits[SEND_BYTE_BITS_LEN];
  size_t cur_bit;
} send_byte_t;

send_byte_t cur_send_byte;

inline void send_byte_init(send_byte_t *st, uint8_t data) {
  st->cur_bit = 0;
  st->bits[0] = TONE_1200HZ; // start bit space
  st->bits[1] = data & 0x01 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[2] = data & 0x02 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[3] = data & 0x04 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[4] = data & 0x08 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[5] = data & 0x10 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[6] = data & 0x20 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[7] = data & 0x40 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[8] = data & 0x80 ? TONE_2200HZ : TONE_1200HZ;
  st->bits[9] = TONE_2200HZ; // stop bit mark
}

inline void send_byte_set_tone(send_byte_t *st) {
  cur_send_tone = st->bits[st->cur_bit];
  st->cur_bit++;
}

inline int send_byte_done(send_byte_t *st) {
  return st->cur_bit >= SEND_BYTE_BITS_LEN;
}


void usart_write(uint8_t data)
{
    USARTD0.DATA = data;
    if(!(USARTD0.STATUS & USART_DREIF_bm)) {
        while(!(USARTD0.STATUS & USART_TXCIF_bm)); // wait for TX complete
    }
    USARTD0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
}


uint8_t read_cal_byte(uint8_t index) {
  uint8_t result;

  /* Load the NVM Command register to read the calibration row. */
  NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
  result = pgm_read_byte(index);

  /* Clean up NVM Command register. */
  NVM_CMD = NVM_CMD_NO_OPERATION_gc;

  return result;
}


static inline void init_oscillator() {
  // enable 32Mhz internal oscillator
  OSC.CTRL |= OSC_RC32MEN_bm;
  // wait for it to be stable
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)); 
  // tell the processor we want to change a protected register
  CCP=CCP_IOREG_gc;
  // and start using the 32Mhz oscillator
  CLK.CTRL=CLK_SCLKSEL_RC32M_gc; 
  // disable the default 2Mhz oscillator
  OSC.CTRL&=(~OSC_RC2MEN_bm);
  // enable 32kHz calibrated internal oscillator
  OSC.CTRL|= OSC_RC32KEN_bm;
  while (!(OSC.STATUS & OSC_RC32KRDY_bm)); 
  // set bit to 0 to indicate we use the internal 32kHz
  // callibrated oscillator as auto-calibration source
  // for our 32Mhz oscillator
  OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc;
  // enable auto-calibration for the 32Mhz oscillator
  DFLLRC32M.CTRL |= DFLL_ENABLE_bm; 
}

static inline void init_usart() {
  // enable clock out on port PC7
  PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
  // set PC7 as output
  PORTC.DIRSET = PIN7_bm;

  // set PD7 as output for TX0
  PORTD.DIRSET = PIN7_bm;
  PORTD.OUTSET = PIN7_bm;
  // remap USARTD0 to PD[7-4]
  PORTD.REMAP |= PORT_USART0_bm;
  // set baud rate 9600: BSEL=12, BSCALE=4
  // as found in table in 
  // Atmel-42005-8-and-16-bit-AVR-Microcontrollers-XMEGA-E_Manual.pdf
  USARTD0.BAUDCTRLA = 12; // BSEL
  USARTD0.BAUDCTRLB = 4 << USART_BSCALE_gp; // BSCALE
  // disable 2X
  USARTD0.CTRLB = USARTD0.CTRLB & ~USART_CLK2X_bm;
  // enable RX and TX
  USARTD0.CTRLB = USARTD0.CTRLB | USART_RXEN_bm | USART_TXEN_bm;
  // enable async UART 8N1
  USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
  USARTD0.CTRLC &= ~USART_SBMODE_bm;
  USARTD0.CTRLD = 0; // No LUT

  // set interrupt level for RX
  USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
}

static inline void init_adc() {
  // read low ADCA calibration byte from NVM signature row into register
  // ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
  // read high ADCA calibration byte from NVM signature row into register
  // ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
  // enable ADC
  ADCA.CTRLA |= 0x1;
  // 12 bit signed resolution and CONVMODE = 1
  ADCA.CTRLB |= ADC_RESOLUTION_8BIT_gc | 0x10;
  // scale to Vcc/2
  ADCA.REFCTRL = ADC_REFSEL_INTVCC2_gc;
  ADCA.PRESCALER = ADC_PRESCALER_DIV4_gc;
  // differential without gain => input signal(A:1) - bias(A:0)
  ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFFWGAINH_gc;
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc | ADC_CH_MUXNEGH_PIN4_gc;
}

/**
 * Setup Timer 4, Port C to be our system event timer. Generates compare
 * interrupts for signal generation and demodulating.
 */
static inline void init_timer() {
  // todo: figure out an appropriete clock scale
  TCC4.CTRLA |= TC4_CLKSEL0_bm;
  // set compare mode
  TCC4.CTRLE |= TC4_CCBMODE0_bm | TC4_CCAMODE0_bm;
  // configure as HIGH level interrupts
  TCC4.INTCTRLB |= TC4_CCBINTLVL0_bm | TC4_CCBINTLVL1_bm;
  TCC4.CCB = 6000;
}

static inline void init_dac() {
  PORTA.DIRSET = PIN2_bm;
  // enable output 0 (PA2) and the DAC
  DACA.CTRLA |= DAC_CH0EN_bm | DAC_ENABLE_bm;
  // channel select implicitly set for CH0 single operation
  // set AVcc as ref
  DACA.CTRLC |= DAC_REFSEL0_bm;
}

static inline void init_interrupts() {
  // Enable PMIC interrupt level low
  PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm;
  // enable interrupts
  sei();
}


int main( void )
{
  init_oscillator();

  // enable clock out on port PC7
  PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
  // set PC7 as output
  PORTC.DIRSET = PIN7_bm;

  // setup data structures
  buf_init(&radio_send_buf);
  buf_init(&radio_recv_buf);

  // setup pins
  // use PD:2 (pin 26) as input
  PORTD.DIR &= ~PIN2_bm;
  PORTD.DIR |= PIN1_bm | PIN0_bm;
  PORTD.OUT &= ~PIN1_bm & ~PIN0_bm;

  init_adc();
  init_timer();
  init_dac();
  init_usart();
  init_interrupts();

  // set PA0 as output
  // PORTA.DIRSET = PIN0_bm;
  // blink LED on PA0 with 1 second on, 1 second off
  // write echo_char on USART on D7; defaults to 42(*)

  while (1) {
    if(PORTD.IN & PIN2_bm) {
      // high: receive mode
      cur_mode = MODE_RECEIVE;
    } else {
      cur_mode = MODE_SEND;
    }

    if(test_data_size == 256) {
      for(int i = 0; i < 256; i++) {
        usart_write(test_data[i]);
      }
      test_data_size = 257;
    }

    // usart_write(echo_char);
    // PORTA.OUTSET = PIN0_bm;
    // _delay_ms( 1000 );
    // usart_write(echo_char);
    // PORTA.OUTCLR = PIN0_bm;
    // _delay_ms( 1000 );
  }
}


static inline void set_next_iter() {
  if(cur_mode == MODE_RECEIVE) {
    // TCC4.CCB += TICKS_9600HZ;
    TCC4.CCB += 3310;
  } else {
    // cur_mode = MODE_SEND
    if(cur_send_tone == TONE_2200HZ) {
      TCC4.CCB += TICKS_2200HZ_BY_32;
    } else {
      TCC4.CCB += TICKS_1200HZ_BY_32;
    }
  }
}


static inline void receive_cycle() {
  // capture from ADC
  ADCA.CTRLA |= 0x4;
  while(!ADCA.CH0.INTFLAGS); // wait for conversion complete flag
  ADCA.CH0.INTFLAGS = 0;

  // save to test buffer
  if(test_data_size < 256) {
    test_data[test_data_size] = ADCA.CH0.RESL;
    test_data_size++;
  }
}


static inline void send_cycle() {
  // write some DAC output
  DACA.CH0DATA = sine_table[sine_table_i];
  // and update increment
  sine_table_i = (sine_table_i + 32);
  if(sine_table_i >= sine_table_size) {
    sine_table_i -= sine_table_size;
  }

  // if(cur_send_mode == SEND_MODE_COLD) {
  //   if(!buf_is_empty(&radio_send_buf)) {
  //     // init sending by loading from buffer if there are bytes
  //     uint8_t data = buf_remove(&radio_send_buf);
  //     send_byte_init(&cur_send_byte, data);
  //     send_byte_set_tone(&cur_send_byte);
  //     cur_send_mode = SEND_MODE_ACTIVE;
  //   }

  //   // ignore otherwise

  // } else {
  //   if(send_byte_done(&cur_send_byte)) {
  //     // try to make a new byte
  //     if(!buf_is_empty(&radio_send_buf)) {
  //       uint8_t data = buf_remove(&radio_send_buf);
  //       send_byte_init(&cur_send_byte, data);
  //       send_byte_set_tone(&cur_send_byte);
  //     } else {
  //       cur_send_mode = SEND_MODE_COLD;
  //     }
  //   } else {
  //     // move to the next bit of this byte
  //     send_byte_set_tone(&cur_send_byte);
  //   }
  // }

  next_bit_switch = TCC4.CCB + TICKS_1200HZ;
}


// USART RX receive interrupt handler
ISR(USARTD0_RXC_vect) {
  if(buf_is_full(&radio_send_buf)) {
  // if(1==0) {
    // ignore and flip the overflow bit
    // usart_write((char) (radio_send_buf.first & 0xFF));
    // usart_write((char) (radio_send_buf.next & 0xFF));
    // usart_write(USARTD0.DATA);
    uint8_t have_to_read = USARTD0.DATA;
    have_to_read = have_to_read;
    PORTD.OUT |= PIN0_bm;
  } else {
    // usart_write(USARTD0.DATA);
    // save to the radio send buf
    buf_add(&radio_send_buf, USARTD0.DATA);
    usart_write(buf_len(&radio_send_buf) & 0xFF);
  }
}


ISR(TCC4_CCB_vect) {
  PORTD.OUT |= PIN1_bm;

  if(cur_mode == MODE_RECEIVE) {
    receive_cycle();
    // send_cycle();
  } else {
    send_cycle();
  }

  set_next_iter(); // important: after to react to new tones

  PORTD.OUT &= ~PIN1_bm;
}
