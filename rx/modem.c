#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define SOUTDDR DDRD
#define SOUTPORT PORTD
#define SOUTPIN PD6 // D6


volatile uint8_t adc_vals[8] = {128, 128, 128, 128, 128, 128, 128, 128};


int main(void) {

  // PORTD &= ~_BV(PD7);
  // DDRD |= _BV(PD7);
  DDRB    |= 0b00011111;          // Set PORTB.0,PORTB.1 as output
  PORTB   |= 0b00011111;          // Initialize PORTB.0,PORTB.1 with logic "one"

  sei();

  // set timer for no scaling Fovr = 16000000/256 = 62500Hz, fast pwm mode
  // TCCR0A |= _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
  // TCCR0B |= _BV(CS00);
  // TIMSK0 |= _BV(TOIE0);
  // OCR0A = 127;

  // set timer 1 for CTC mode, no scaling Fovr = 16000000/256 = 62500Hz
  // TIFR1 = TIFR1; // clear interrupt flags
  // // TCCR1A |= 0;
  // TCCR1A |= _BV(COM1B0);
  // TCCR1B |= _BV(CS11) | _BV(CS10) | _BV(WGM12);
  // OCR1B = 156;

  // set adc for timer/counter 1 compare match b
  // enable adc, enable auto trigger/interrupt, set clock div to 32 for 500kHz
  ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0);
  ADCSRB |= _BV(ADTS2) | _BV(ADTS0);
  // ADCSRB |= _BV(ADTS1) | _BV(ADTS0);
  ADMUX |= _BV(REFS0) | _BV(ADLAR); // use AVcc, A0 as input
  DIDR0 |= _BV(ADC0D);

  // init timer
  TIFR0    =  TIFR0;        // Clear all TCNT0 interrupt flags by writing '1' to them

  TCCR0A   =  0x02;         // CTC Mode

  TCCR0B   =  0x03;         // Configure the timer base clock as required. (here 0.015625 MHz(64 microseconds) which determines the 1 TCNT0 count equivalent time )
                            // Enable the TCNT0, prescale by 16 (change the prescaler value as required)
                            // For Example if we use internal 8 MHz clock then 8 MHz/8(CLKDIV8 FUSE) = 1 MHz but 1 MHz/64 = 0.015625 MHz timer base frequency

  OCR0A    =  156;          // Configure the timer top count as required. (here 156 ;so timer will run every 156 * 64 =9984 microsecond)
                            // 1 count = 1/0.015625 MHz = 64 Microseconds; 156 counts = 156 * 64 = 9984 microseconds
                            // So approximately Every 10 millisecond (9984 microseconds auto reload timer ISR will be generated.

  while(1) {
    // PORTD |= _BV(PD7);
    // _delay_ms(10);
    // PORTD &= ~_BV(PD7);
    // _delay_ms(10);
    PORTB = (PORTB & ~0x08) | ((TCNT0 & 0x1) << 3);
    PORTB = (PORTB & ~0x10) | ((TCNT1 & 0x1) << 4);
  }
  return 0;
}


ISR(ADC_vect) {
  PORTB &= ~_BV(PB1);

  ADCSRA  |=(1 << ADIF);  // Acknowledge the interrupt flag
  TIFR0    =  TIFR0;      // Acknowledge the Timer0 Compare MatchA event flag
  TIFR1 = TIFR1;
  adc_vals[0] = ADCH;

  _delay_us(20);
  PORTB |= _BV(PB1);
}
