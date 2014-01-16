#include <stdio.h>
#include <avr/io.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include <math.h>

void GenerateArbWave12(int *data, int len, unsigned long int freq);
void SetWaveFreq(unsigned long int freq);
void LoadSineWave(int len);
void Config32MHzClock(void);

volatile int data12[100];
volatile int gWaveNumSamp=50;

int main(void)
{
  // int Reading;

  Config32MHzClock();

  CLK.PSCTRL = 0x00; // no division on peripheral clock

  // enable clock out on port PC7
  PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC7_gc;
  // set PC7 as output
  PORTC.DIRSET = PIN7_bm;
  TCC4.CTRLA = 4;

// setup DAC output on PORTA:2 as GND reference
  PORTA.DIRSET = PIN2_bm;
  PORTA.OUT &= 0xFB;

// setup ADC input on PORTA:0-3 (0=hi, 1=hi, 2=samp, 3=gnd)
// and power PORTA:1 to create voltage divider
  // PORTA.DIR = 0xB;
  // PORTA.OUT = 0x3;

// setup adc for single ended one shot mode
  // ADCA.CTRLA |= 0x1;       // enable adc
  // ADCA.CTRLB = 0x4;        // set 8 bit conversion
  // ADCA.CH0.CTRL = 0x1;     // single ended
  // ADCA.CH0.MUXCTRL = 0x10; // PORTA:2
  // ADCA.REFCTRL = 0x20;     // reference is PORTA:0
  // ADCA.PRESCALER = 0x5;    // peripheral clk/128

  LoadSineWave(gWaveNumSamp);

  // startup in 10kHz
  GenerateArbWave12(data12,gWaveNumSamp*2,10000);

  while(1)
  {
  // read adc to determine waveform freq
 //    ADCA.CTRLA |= 0x4;               // start conversion ch0
 //    while(!ADCA.CH0.INTFLAGS);       // wait for conversion complete flag
 //    ADCA.CH0.INTFLAGS = 0;           // clear int flags  
	// Reading = ADCA.CH0RESL;          // read 8 bit value from POT
 //    SetWaveFreq((Reading*Reading)+1); // set freq
    _delay_ms(100); 
  };
  
};

void LoadSineWave(int len)
{
int i;

  for(i=0;i<len;i++)
  {
    data12[i]=((sin((2.0/(float)len)*(float)i*M_PI)*0.5 + 0.5)*4095);
  };

};

// void SetWaveFreq(unsigned long int freq)
// {
//     TCD5.PER = F_CPU/freq/gWaveNumSamp/4;
// };

void GenerateArbWave12(int *data, int len, unsigned long int freq)
{

EVSYS.CH1MUX = 0xD0;    // CH1 = TCD5 overflow
TCD5.CTRLA = 0x03;      // Prescaler: clk/4
TCD5.PER   = F_CPU/(len/2)/freq/4;        // 31=1MHz,63=500K,127=250K
DACA.CTRLA = 0x05;      // Enable DACA and CH0
DACA.CTRLB = 0x01;  // CH0 auto triggered by an event (CH1)
DACA.CTRLC = 0x08;  // Use AVCC (3.3V), left adjust
DACA.EVCTRL = 0x01; // Event CH1 triggers the DAC Conversion
DACA.TIMCTRL = 0x50;// Minimum 32 CLK between conversions
EDMA.CTRL = 0x80 | EDMA_CHMODE_STD0_gc;    // Enable, single buffer, round robin
EDMA.CH0.ADDRCTRL = 0xD9;// Reload, Increment source
EDMA.CH0.TRIGSRC= 0x15;  // DACA CH0 is trigger source
EDMA.CH0.TRFCNTL = len & 0xFF;   // Buffer is len bytes
EDMA.CH0.TRFCNTH = (len >> 8) & 0xFF;
EDMA.CH0.ADDRL  =(((uint16_t)data)>>0*8) & 0xFF; // SRCADDRL
EDMA.CH0.ADDRH  =(((uint16_t)data)>>1*8) & 0xFF; // SRCADDRH
EDMA.CH0.DESTADDRL =(((uint16_t)(&DACA.CH0DATA))>>0*8)&0xFF;
EDMA.CH0.DESTADDRH =(((uint16_t)(&DACA.CH0DATA))>>1*8)&0xFF;
EDMA.CH0.CTRLA = 0xA5;   // Enable, repeat, 1 byte, single 
};



void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
//  CCP = CCP_IOREG_gc;
//  CLK.PSCTRL = 0x02; // peripheral clk = sysclk/4, forces sys also /4??
};
