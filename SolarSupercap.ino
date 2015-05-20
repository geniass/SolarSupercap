#include <PWM.h>


#include <avr/power.h>
#include <avr/wdt.h>

// PARAMETERS TO BE TUNED
#define MAX_BOOST_DUTY 178 // 188/255 = 74%
#define MIN_BOOST_DUTY 77 // 77/255 = 30%
#define MAX_MPPT_DUTY 170
#define MIN_MPPT_DUTY 0
#define BOOST_V_OFFSET 0.3
#define MPPT_V_OFFSET 0
#define MPPT_I_OFFSET 0

// BOOST CONSTANTS
#define BOOST_VOLTAGE 5
#define BOOST_DIVIDER_RATIO (10e3)/(10e3+15e3)
#define EPSILON_BITS 60 // 60 bits at 3.3V == 0.2V
#define EPSILON_V 0.1
#define BOOST_PWM_PIN 3
#define BOOST_V_ADC 0


// MPPT CONSTANTS
#define MPPT_PWM_PIN 5
#define MPPT_V_ADC 1
#define MPPT_I_ADC 2
#define MPPT_PERIOD 100  // number of main loops before MPPT runs
#define MPPT_VOLTAGE_STEP 0.2
#define MPPT_DUTY_STEP 1
#define MPPT_CURRENT_FACTOR 10
#define MPPT_DIVIDER_RATIO (10e3)/(10e3+15e3)


// BOOST
volatile int boost_duty = 128;
int boost_v_target = 5;
float boost_v = 5;

//MPPT
int mppt_duty = 128;
float mppt_v = 5;
float mppt_i = 0.2;
float mppt_target_v = 5;
float mppt_target_i = 0.2;
int mppt_period = MPPT_PERIOD;

int ledPin = 11;      // select the pin for the LED

volatile int analogVals[2];
volatile int readFlag = 0;
volatile int prevADC = 0;
volatile int currADC = 1;

float VCC = 5;


void setup() {
  wdt_enable(WDTO_2S);
  
  // DISABLE SPI AND I2C
  power_spi_disable();
  power_twi_disable(); 
  
  Serial.begin(115200);
  
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  
  InitTimersSafe();
 

  pinMode(BOOST_PWM_PIN,OUTPUT);
  bool success = SetPinFrequencySafe(BOOST_PWM_PIN, 31250);
  if(success){
    Serial.println("Set pin 3 to 31250Hz");
  } else {
    Serial.println("Error setting up PWM pin 3!!");
  }
  pwmWrite(BOOST_PWM_PIN,128);
  
  pinMode(MPPT_PWM_PIN,OUTPUT);
  setPwmFrequency(MPPT_PWM_PIN,1);
  analogWrite(MPPT_PWM_PIN,138);
 
 
//  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  OCR1A = 500;            // compare match register 8MHz/8/1000Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler 
 // TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei();

//  ADCSetup();
  
  VCC = read_vcc()/1000.;
  Serial.print("VCC: ");
  Serial.println(VCC);
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
    
    int temp,tempI;
    
    temp = analogRead(BOOST_V_ADC);
    boost_v = get_boost_voltage(temp,VCC);
    //Serial.println(boost_v);
    boost_pid(BOOST_VOLTAGE, boost_v);
    
    tempI = analogRead(MPPT_I_ADC);
    temp = analogRead(MPPT_V_ADC);
    mppt_v = get_mppt_voltage(temp, VCC);
    mppt_i = get_mppt_current(tempI, VCC);
    
    if (mppt_period == 0)
    {
      Serial.print("mppt_v: ");
      Serial.println(mppt_v);
      Serial.print("mppt_i: ");
      Serial.println(mppt_i);
      // Don't run MPPT every iteration
      mppt_inccond(mppt_v, mppt_i);
      
      mppt_period = MPPT_PERIOD;
    } else {
      mppt_period--;
    }
    
    mppt_pid(mppt_target_v, mppt_target_i, mppt_v, mppt_i);
  }   
}

// Read the internal reference (1.1V) relative to VCC
// https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
long read_vcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = (1024L*1100L) / result; // Back-calculate AVcc in mV
  Serial.println(result);
  return result;
}

// BOOST CONTROL
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
    pwmWrite(BOOST_PWM_PIN, boost_duty);
  }
}

//MPPT CONTROL
void mppt_inccond(float v, float i)
{
  static long prev_p = 0;
  static long prev_v = 0;
  static long prev_i = 0;
  
  long mV = (long)(v*1000);
  long mA = (long)(i*1000);
  
  long power = mV * mA;
  long delta_p = power - prev_p;
  long delta_v = mV - prev_v;
  long delta_i = mA - prev_i;
  
  if (delta_v != 0) {
    long d_power = delta_p / delta_v;
    // check if dP/dV is positive or negative
    if (d_power > 0) {
      mppt_target_v += MPPT_VOLTAGE_STEP;
    } else {
      mppt_target_v -= MPPT_VOLTAGE_STEP;
    }
  } else {
    //voltage hasn't changed, but current has
    if (delta_i > 0) {
      mppt_target_v += MPPT_VOLTAGE_STEP;
    } else {
      mppt_target_v -= MPPT_VOLTAGE_STEP;
    }
  }
  prev_v = mV;
  prev_i = mA;
  prev_p = power;
}

void mppt_pid(float target_v, float target_i, float v, float i)
{
  float delta_v = v - target_v;
 // Serial.println(delta_v);
  if (delta_v > EPSILON_V)
  {
    // buck voltage > target; decrease duty cycle
    set_mppt_duty(mppt_duty - 1);
  } else if (delta_v < -EPSILON_V)
  {
    // buck voltage < target; increase duty cycle
    set_mppt_duty(mppt_duty + 1);
  }
}

void set_mppt_duty(int duty)
{
  if (duty > MAX_MPPT_DUTY)
  {
    Serial.println("Attempted to set mppt duty cycle too high!");
    mppt_duty = MAX_MPPT_DUTY;
  } else if (duty < MIN_MPPT_DUTY)
  {
    Serial.println("Attempted to set mppt duty cycle too low!");
    mppt_duty = MIN_MPPT_DUTY;
  } else
  {
    mppt_duty = duty;
    analogWrite(MPPT_PWM_PIN, mppt_duty);
  }
}

// SENSOR FUNCTIONS
float get_boost_voltage(int adc, float vref)
{
  // voltage divider ratio is (10e3)/(10e3+15e3)
  return (vref * adc)/(BOOST_DIVIDER_RATIO * 1023) + BOOST_V_OFFSET; 
}

float get_mppt_voltage(int adc, float vref)
{
  // voltage divider ratio is (10e3)/(10e3+15e3)
  return (vref * adc)/(MPPT_DIVIDER_RATIO * 1023) + MPPT_V_OFFSET; 
}

float get_mppt_current(int adc, float vref)
{
  return (vref*adc)/(MPPT_CURRENT_FACTOR*1023) + MPPT_I_OFFSET;
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

