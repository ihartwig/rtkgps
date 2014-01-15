// (C) 2013 Joost Yervante Damad <joost@damad.be>
// License: GPL3

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

static volatile uint8_t echo_char = 42;
static volatile int cur_recv_signal = 0; // 12 bit value from the adc


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
  ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFFWGAINL_gc;
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc | ADC_CH_MUXNEGL_PIN0_gc;
}

/**
 * Setup Timer 4, Port C to be our system event timer. Generates compare
 * interrupts for signal generation and demodulating.
 */
static inline void init_timer() {
  // todo: figure out an appropriete clock scale
  TCC4.CTRLA |= TC4_CLKSEL0_bm | TC4_CLKSEL1_bm | TC4_CLKSEL2_bm;
  // set compare mode
  TCC4.CTRLE |= TC4_CCBMODE0_bm | TC4_CCAMODE0_bm;
  // configure as HIGH level interrupts
  TCC4.INTCTRLB |= TC4_CCBINTLVL0_bm | TC4_CCBINTLVL1_bm |
                   TC4_CCAINTLVL0_bm | TC4_CCAINTLVL1_bm;
  TCC4.CCA = 3000;
  TCC4.CCB = 0;
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

  init_adc();

  init_timer();

  init_usart();

  init_interrupts();

  // set PA0 as output
  PORTA.DIRSET = PIN0_bm;
  // blink LED on PA0 with 1 second on, 1 second off
  // write echo_char on USART on D7; defaults to 42(*)
  while (1) {
    _delay_ms(100);
    // usart_write(echo_char);
    // PORTA.OUTSET = PIN0_bm;
    // _delay_ms( 1000 );
    // usart_write(echo_char);
    // PORTA.OUTCLR = PIN0_bm;
    // _delay_ms( 1000 );
  }
}


// USART RX receive interrupt handler
ISR(USARTD0_RXC_vect) {
  echo_char = USARTD0.DATA;
}


ISR(TCC4_CCA_vect) {
  // set next time
  TCC4.CCA += 5000;

  // capture from ADC
  ADCA.CTRLA |= 0x4;
  while(!ADCA.CH0.INTFLAGS);                 // wait for conversion complete flag
  ADCA.CH0.INTFLAGS = 0;

  usart_write(ADCA.CH0RESL);
}


ISR(TCC4_CCB_vect) {

}


// interrupt should be called after each DMA transaction is complete
// ISR(ADCA_CH0_vect)
// {
//   cur_recv_signal = ADCA.CH0RES;
//   echo_char = (char) (cur_recv_signal >> 4);
// };
