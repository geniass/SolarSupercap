
#include <avr/power.h>
#include <avr/wdt.h>

// BOOST CONSTANTS
#define BOOST_VOLTAGE 5
#define EPSILON_BITS 60 // 60 bits at 3.3V == 0.2V
#define MAX_BOOST_DUTY 204 // 204/255 = 80%
#define MIN_BOOST_DUTY 77 // 77/255 = 30%

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 11;      // select the pin for the LED

volatile int analogVals[2];
volatile int readFlag = 0;
volatile int prevADC = 0;
volatile int currADC = 1;

volatile int boost_duty = 128;

void setup() {
  wdt_enable(WDTO_2S);
  
  // DISABLE SPI AND I2C
  power_spi_disable();
  power_twi_disable(); 
  
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin,HIGH); 
  
  pinMode(5,OUTPUT);
  setPwmFrequency(5,1);
  analogWrite(5,173);
  
//  pinMode(6,OUTPUT);
//  setPwmFrequency(6,1);
//  analogWrite(6,200);
 

  Serial.begin(9600);

  ADCSetup();
}

void loop() {
  wdt_reset();
  
  // Boost converter control
  int boost_voltage = analogRead(1);
  int delta_v = boost_voltage - BOOST_VOLTAGE;
  if (delta_v > EPSILON_BITS)
  {
    // boost voltage > target; decrease duty cycle
    set_boost_duty(boost_duty - 1);
  } else if (delta_v < EPSILON_BITS)
  {
    // boost voltage < target; increase duty cycle
    set_boost_duty(boost_duty + 1);
  }
  
  
  
  // Check to see if the value has been updated
  if (readFlag == 1){
    if(prevADC==1){
//      float voltage = analogVals[prevADC]*5./255.;
//      int val = map(analogVals[prevADC]+20,0,900,0,255);
//      analogWrite(5,val);
//      Serial.println(voltage);
    }


    readFlag = 0;
  }    
}

void set_boost_duty(int duty)
{
  if (duty > MAX_BOOST_DUTY)
  {
    Serial.println('Attempted to set boost duty cycle too high!');
    boost_duty = MAX_BOOST_DUTY;
  } else if (duty < MIN_BOOST_DUTY)
  {
    Serial.println('Attempted to set boost duty cycle too low!');
    boost_duty = MIN_BOOST_DUTY;
  } else
  {
    boost_duty = duty;
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

