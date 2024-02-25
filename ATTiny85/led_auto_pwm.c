/*
             |||
            (o o)
 +-------oOO-{_}-OOo--------+
 |                          |
 |   © 2024 Ola Tuvesson    |
 |   clickworkorange Ltd    |
 |                          |
 | info@clickworkorange.com |
 |                          |
 +--------------------------+

*/
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>
#include "led_auto_pwm.h"

// Array holding last ACNT samples
static uint16_t samples[ACNT] = {[0 ... ACNT-1] = 0};
// Pointer to last/current sample
static  uint8_t samptr = 0; 
// Variable holding actual sampling delay
static uint16_t delay = ADCD;

uint16_t min(uint16_t a, uint16_t b) {
  // Return the lowest of two values
  return a < b ? a : b;
}

uint16_t max(uint16_t a, uint16_t b) {
  // Return the highest of two values
  return a > b ? a : b;
}

uint16_t avg(uint16_t arr[], uint8_t size) {
  // Return average of array values
  float average = 0.0;
  for(int i = 0; i < size; i++){
     average = average + arr[i];
  }
  return (int)(average / size);
}

uint16_t map(uint16_t val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  // Map a value from one range of integers to another
  double slope = 1.0 * (out_max - out_min) / (in_max - in_min);
  return out_min + slope * (val - in_min);
}

void delay_us(int d) {
  // _delay_us only accepts a constant, so we comply
  for (int i = 0; i < (d / 10); i++) {
    // But looping also takes time, so we do bigger chunks
    _delay_us(10);
  }
}

void addSample(uint16_t sample) {
  // Add sample to samples and increment pointer, wrapping at ACNT
  samples[samptr] = sample;
  samptr++;
  if (samptr == ACNT) {
    samptr = 0;
  }
}

void initADC(void) {
  // Enable ADC w. prescaler 128, conversion complete interrupt enabled
  ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
  // Select internal ref (2.56 V), ADC channel 2
  ADMUX = (1<<REFS2)|(1<<REFS1)|(1<<MUX1);
  // Disable digital input buffer on ADC2
  DIDR0 = (1<<ADC2D);
}

void initPWM(void) {
  // Timer0B, Fast PWM Mode 3, Top = 0xFF, Inverted
  TCCR0A = (1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00); 
  // Prescaler 256 (120Hz @ 8MHz)
  TCCR0B = (1<<CS02); //|(1<<CS00); 
  // TIMER1_COMPB_vect ISR on TCNT0 matches OCR0B 
  TIMSK = (1<<OCIE0B); 
  // Port B1 PWM out
  DDRB |= (1<<PB1);
}

ISR(TIMER0_COMPB_vect) {
  // Timer1 reached TOP
  delay_us(delay);
  // Indicate ADC conversion start, PB3 high
  PORTB |= (1<<PB3);
  // Start ADC conversion
  ADCSRA |= (1<<ADSC);
}

ISR(ADC_vect) {
  // Indicate ADC conversion end, PB3 low
  PORTB &= ~(1<<PB3);
  // ADC conversion completed, add result to samples
  addSample(ADC);
}

int main(void) {
  // TODO: Merge ATMega328 & ATTiny85 code
  // TODO: Verify internal 2.56 V reference
  // TODO: Test logarithmic OPAMP 
  // TODO: Add button to set max level 
  // TODO: Explore using Noise Reduction Mode
  sei();
  initADC();
  initPWM();
  while(1) {
    // Filter average and set PWM
    int pwm = map(avg(samples, ACNT), AMIN, AMAX, DMIN, DMAX);
    if (pwm > OCR0B) {
      int step = (pwm - OCR0B) / 10;
      if (step < 1) {
        step = 1;
      }
      OCR0B += step;
    } else if (pwm < OCR0B) {
      int step = (OCR0B - pwm) / 10;
      if (step < 1) {
        step = 1;
      }
      OCR0B -= step;
    }
    // Shorten sampling delay for higher voltages by subtracting OCR0B x 2
    delay = ADCD - (OCR0B*2);
  }
}
