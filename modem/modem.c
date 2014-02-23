#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define SOUTDDR DDRB
#define SOUTPORT PORTB
#define SOUTPIN PB1 // D9

void inline bit_bang_toggle(double on_time) {
  SOUTPORT |= _BV(SOUTPIN);
  _delay_us(on_time);
  SOUTPORT &= ~_BV(SOUTPIN);
  _delay_us(16.0 - on_time); // was 35
}

int main(void) {
  // set output for sine generation, start off
  SOUTPORT |= _BV(SOUTPIN);
  SOUTDDR |= _BV(SOUTPIN);

  while(1) {
    // bit_bang_toggle(17.5000);
    // bit_bang_toggle(22.0293);
    // bit_bang_toggle(26.2500);
    // bit_bang_toggle(29.8744);
    // bit_bang_toggle(32.6554);
    // bit_bang_toggle(34.4037);
    // bit_bang_toggle(35.0000);
    // bit_bang_toggle(34.4037);
    // bit_bang_toggle(32.6554);
    // bit_bang_toggle(29.8744);
    // bit_bang_toggle(26.2500);
    // bit_bang_toggle(22.0293);
    // bit_bang_toggle(17.5000);
    // bit_bang_toggle(12.9707);
    // bit_bang_toggle( 8.7500);
    // bit_bang_toggle( 5.1256);
    // bit_bang_toggle( 2.3446);
    // bit_bang_toggle( 0.5963);
    // bit_bang_toggle(      0);
    // bit_bang_toggle( 0.5963);
    // bit_bang_toggle( 2.3446);
    // bit_bang_toggle( 5.1256);
    // bit_bang_toggle( 8.7500);
    // bit_bang_toggle(12.9707);

    bit_bang_toggle( 8.0000);
    bit_bang_toggle( 8.9643);
    bit_bang_toggle( 9.9145);
    bit_bang_toggle(10.8368);
    bit_bang_toggle(11.7178);
    bit_bang_toggle(12.5445);
    bit_bang_toggle(13.3050);
    bit_bang_toggle(13.9881);
    bit_bang_toggle(14.5839);
    bit_bang_toggle(15.0836);
    bit_bang_toggle(15.4801);
    bit_bang_toggle(15.7675);
    bit_bang_toggle(15.9417);
    bit_bang_toggle(16.0000);
    bit_bang_toggle(15.9417);
    bit_bang_toggle(15.7675);
    bit_bang_toggle(15.4801);
    bit_bang_toggle(15.0836);
    bit_bang_toggle(14.5839);
    bit_bang_toggle(13.9881);
    bit_bang_toggle(13.3050);
    bit_bang_toggle(12.5445);
    bit_bang_toggle(11.7178);
    bit_bang_toggle(10.8368);
    bit_bang_toggle( 9.9145);
    bit_bang_toggle( 8.9643);
    bit_bang_toggle( 8.0000);
    bit_bang_toggle( 7.0357);
    bit_bang_toggle( 6.0855);
    bit_bang_toggle( 5.1632);
    bit_bang_toggle( 4.2822);
    bit_bang_toggle( 3.4555);
    bit_bang_toggle( 2.6950);
    bit_bang_toggle( 2.0119);
    bit_bang_toggle( 1.4161);
    bit_bang_toggle( 0.9164);
    bit_bang_toggle( 0.5199);
    bit_bang_toggle( 0.2325);
    bit_bang_toggle( 0.0583);
    bit_bang_toggle(      0);
    bit_bang_toggle( 0.0583);
    bit_bang_toggle( 0.2325);
    bit_bang_toggle( 0.5199);
    bit_bang_toggle( 0.9164);
    bit_bang_toggle( 1.4161);
    bit_bang_toggle( 2.0119);
    bit_bang_toggle( 2.6950);
    bit_bang_toggle( 3.4555);
    bit_bang_toggle( 4.2822);
    bit_bang_toggle( 5.1632);
    bit_bang_toggle( 6.0855);
    bit_bang_toggle( 7.0357);
  }
  return 0;
}
