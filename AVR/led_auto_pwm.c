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
#include "lcd.h"

// Array holding last ACNT samples
static uint16_t samples[ACNT] = {[0 ... ACNT-1] = 0};
// Pointer to last/current sample
static  uint8_t samptr = 0; 

void initMCU(void) {
  // Enable ADC Noise reduction mode
  // SMCR = (1<<SM0|(1<<SE));
}

void initADC(void) {
  // Enable ADC w. prescaler 128, conversion complete interrupt enabled
  ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
  // Select external ref, ADC channel 0
  ADMUX = 0x0;
  // Disable digital input buffer on ADC0
  DIDR0 = (1<<ADC0D);
}

void initPWM(void) {
  // Fast PWM Mode 7, Top = 0x03FF
  TCCR1A = (1<<COM1A1)|(1<<WGM11)|(1<<WGM10); 
  // Prescaler 64 (120Hz @ 8MHz)
  TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10); 
  // TIMER1_COMPA_vect ISR on TCNT1 matches OCR1A 
  TIMSK1 = (1<<OCIE1A); 
  // Port B1 PWM out
  DDRB |= (1<<PB1);
}

void printValues(uint16_t val1, uint16_t val2) {
  // Print values on OLED display
  char buf1[5];
  char buf2[5];
  // Convert the values to strings (base10)
  sprintf(buf1, "%d", val1);
  sprintf(buf2, "%d", val2);
  lcd_gotoxy(0,0);
  lcd_puts(buf1);
  lcd_puts("    ");
  lcd_gotoxy(0,5);
  lcd_puts(buf2);
  lcd_puts("    ");
}

uint16_t min(uint16_t a, uint16_t b) {
  // Return the lowest of two values
  return a < b ? a : b;
}

uint16_t max(uint16_t a, uint16_t b) {
  // Return the highest of two values
  return a > b ? a : b;
}

uint16_t avg(void) {
  // Return average of ACNT samples as int16
  float average = 0.0;
  for(int i = 0; i < ACNT; i++){
     average = average + samples[i];
  }
  return (int)(average / ACNT);
}

void addSample(uint16_t sample) {
  // Add sample to samples and increment pointer, wrapping at ACNT
  samples[samptr] = sample;
  samptr++;
  if (samptr == ACNT) {
    samptr = 0;
  }
}

ISR(TIMER1_COMPA_vect) {
  // Timer1 reached TOP
  _delay_us(ADCD);
  // indicate ADC conversion start
  PORTB ^= (1<<PB0);
  // Start ADC conversion
  ADCSRA |= (1<<ADSC);
}

ISR(ADC_vect) {
  // indicate ADC conversion end
  PORTB ^= (1<<PB0);
  // ADC conversion completed, add result to samples[]
  addSample(ADC);
}

int main(void) {
  // TODO: Explore using Noise Reduction Mode
  // TODO: Control individual LEDs on/off
  // TODO: Temperature compensation
  // TODO: Use multiple ADC channels?
  initMCU();
  initADC();
  initPWM();
  lcd_init(LCD_DISP_ON);
  lcd_charMode(DOUBLESIZE);
  sei();
  while(1) {
    // Filter average 
    int pwm = max(min(avg(), DMAX), DMIN);
    if (pwm > OCR1A) {
      int step = (pwm - OCR1A) / 10;
      if (step < 1) {
        step = 1;
      }
      OCR1A += step;
    } else if (pwm < OCR1A) {
      int step = (OCR1A - pwm) / 10;
      if (step < 1) {
        step = 1;
      }
      OCR1A -= step;
    } 
    // Print latest sample & average on OLED
    printValues(samples[samptr], OCR1A);
    // Set PWM duty to sample average
    // OCR1A = pwm; 
    // Wait a moment
    //_delay_ms(UPDD);
  }
}
