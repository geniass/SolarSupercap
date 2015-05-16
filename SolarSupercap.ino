#include <PWM.h>


#include <avr/power.h>
#include <avr/wdt.h>

// BOOST CONSTANTS
#define BOOST_VOLTAGE 5
#define BOOST_DIVIDER_RATIO (10e3)/(10e3+15e3)
#define EPSILON_BITS 60 // 60 bits at 3.3V == 0.2V
#define EPSILON_V 0.1
#define MAX_BOOST_DUTY 178 // 188/255 = 74%
#define MIN_BOOST_DUTY 77 // 77/255 = 30%
#define BOOST_PWM_PIN 3
#define BOOST_V_OFFSET 0.1

volatile int boost_duty = 128;
int boost_v_target = 5;
float boost_v = 5;

int ledPin = 11;      // select the pin for the LED

volatile int analogVals[2];
volatile int readFlag = 0;
volatile int prevADC = 0;
volatile int currADC = 1;



void setup() {
  wdt_enable(WDTO_2S);
  
  // DISABLE SPI AND I2C
  power_spi_disable();
  power_twi_disable(); 
  
  Serial.begin(115200);
  
  InitTimersSafe();
  
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);

  pinMode(3,OUTPUT);
  bool success = SetPinFrequencySafe(3, 41250);
  if(success){
    Serial.println("Set pin 3 to 31250Hz");
  }
  pwmWrite(3,128);
  
//  pinMode(BOOST_PWM_PIN,OUTPUT);
//  setPwmFrequency(BOOST_PWM_PIN,1);
//  analogWrite(BOOST_PWM_PIN,138);
 
 
//  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  OCR1A = 500;            // compare match register 8MHz/8/1000Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler 
 // TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei();

  ADCSetup();
}

//ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
//{
//  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin
//}

void loop() {
  wdt_reset();
  
  if (TIFR1 & (1 << OCF1A))  {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
    TIFR1 &= ~(1 << OCF1A);
    
    float v = analogRead(0);
    Serial.println(v);
    float boost_v = get_boost_voltage(v,5);
    Serial.println(boost_v);
  //  Serial.println(boost_v);
  //  Serial.println(boost_duty);
    boost_pid(BOOST_VOLTAGE, boost_v);
  }
  
 // int v = analogRead(0);
  //Serial.println(v);
 // Serial.println(get_boost_voltage(v,5));
  
  // Boost converter control
//  int boost_voltage = analogRead(1);
//  int delta_v = boost_voltage - BOOST_VOLTAGE;
//  if (delta_v > EPSILON_BITS)
//  {
//    // boost voltage > target; decrease duty cycle
//    set_boost_duty(boost_duty - 1);
//  } else if (delta_v < EPSILON_BITS)
//  {
//    // boost voltage < target; increase duty cycle
//    set_boost_duty(boost_duty + 1);
//  }
   
}

void boost_pid(float target, float v)
{
  float delta_v = v - BOOST_VOLTAGE;
 // Serial.println(delta_v);
  if (delta_v > EPSILON_V)
  {
    // boost voltage > target; decrease duty cycle
    set_boost_duty(boost_duty - 1);
  } else if (delta_v < -EPSILON_V)
  {
    // boost voltage < target; increase duty cycle
    set_boost_duty(boost_duty + 1);
  }
}

float get_boost_voltage(int adc, float vref)
{
  // voltage divider ratio is (10e3)/(10e3+15e3)
  return (vref * adc)/(BOOST_DIVIDER_RATIO * 1023) + BOOST_V_OFFSET; 
}

void set_boost_duty(int duty)
{
  if (duty > MAX_BOOST_DUTY)
  {
    Serial.println("Attempted to set boost duty cycle too high!");
    boost_duty = MAX_BOOST_DUTY;
  } else if (duty < MIN_BOOST_DUTY)
  {
    Serial.println("Attempted to set boost duty cycle too low!");
    boost_duty = MIN_BOOST_DUTY;
  } else
  {
    boost_duty = duty;
    Serial.println(boost_duty);
    pwmWrite(BOOST_PWM_PIN, boost_duty);
  }
}

// Interrupt service routine for the ADC completion
//ISR(ADC_vect){
//
//  // Done reading
//  readFlag = 1;
//
//  // Must read low first
//  //analogVal = ADCL | (ADCH << 8);
//  analogVals[currADC] = ADCH;
//  
//  prevADC = currADC;
//  currADC = currADC==0? 1 : 0;
//  
//  ADMUX &= B11110000;
//  ADMUX |= currADC;
//  // Not needed because free-running mode is enabled.
//  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
//  ADCSRA |= B01000000;
//}          

