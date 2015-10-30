// PID Control Project
// Input - A0
// PWMout - 10

#include <PID_v1.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


#define I2C_ADDR    0x27  // Define I2C Address 
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C       lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

//Define Variables we'll use with PID control
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=20, aggKi=2, aggKd=5;
double consKp=20, consKi=2, consKd=5; 
int smooth;
int counter=0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

unsigned long previousMillis = 0; // initialize internal clock

void setup()
{
   lcd.begin (20,4,LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE); // init the backlight
       lcd.setBacklight(HIGH);
  
    Serial.begin(9600);
  //initialize the variables we're linked to
  Input = analogRead(A0);
  Setpoint = 27; // Our setpoint

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //for smooth startup
for(smooth=0; smooth<=250; smooth++)
{
analogWrite(10, smooth);
delay(2);
}
  
}

void loop()
{

  lcd.setCursor ( 0, 0 );        // go to first line

  
  Input = analogRead(A0)*0.052551; // 0.052551 value was determined by 
  
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<1)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
// note: since our Kp, Ki, Kd values for both modes are the same, it will be operating in single mode
  
  myPID.Compute();

  analogWrite(10,Output);

// for delayed output
  unsigned long currentMillis = millis(); // time since device started up
  if (currentMillis - previousMillis >= 1000) // 1000- interval of 1 second. If the difference is >1s, print values
  {
    previousMillis = currentMillis; 
    Serial.print(Input);
    Serial.print(" C");
    Serial.println();
    Serial.print(Output);
    Serial.println();
    lcd.print(Input);
      lcd.setCursor ( 0, 1 );        // go to second line
      lcd.print(Setpoint);
  }
}





