
//#include <AutoPID.h>
#include <max6675.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <PID_v1.h>
#include <QueueArray.h>

// Buttons for direct control of parameters and settings:
int b_sel = 4; //Set pin for select button here
int b_inc = 5; //Set pin for incremet button here
int b_dec = 6; //Set pin for decrement button here
// Variables to hold the button states
bool sel = LOW;
bool inc = LOW;
bool dec = LOW;
// Interrupt pin for buttons
int b_IRQ = 2; //Set pin for Interrupt ReQuest (IRQ-Pin) here! Either pin 2 or 3 for Uno.

// Potentiometer for direct control of temperature
int pot_temp = A0; //Set pin for Potentiometer here


//MAX6675 for reading THERMOCOUPLE
int thermoDO = 11;
int thermoCS = 12;
int thermoCLK = 13;
//Create Reading for MAX6675
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Temperature Gloabals
double temp;  //holds actual Temp  
double settemp;  //holds Settemp  
//Variables for ramping filter 
QueueArray <double> lastTemp;  //holds last few temp values to get an average
int queueSize = 3;  //sets amount of values for average - put 0 for not using this filter. Higher Values create more lag!
double averageTemp;   //New smoothed Temp Variable to be used with the PID
double SmoothSettemp; //New Settemp to be used with the PID
//varibales for smoothing filter
double Smoothing_Factor_avT = 0.5; // Smoothing Filter put 0 for not using this filter, the closer to 1 the stronger the filter. Higher Values create more lag!
double Smoothing_Factor_SetT = 0.9; //Factor for linear Smoothing


//DISPLAYS
LiquidCrystal_I2C lcd1(0x26,16,2); //Adress, chars, rows
LiquidCrystal_I2C lcd2(0x27,16,2);

//Buttons for PID finetuning:
bool AState1 = false, AState2 = false, AState3 = false; // Variables to hold the state of the Analog input
int Selector = 0; // for choosing wich value to change

//PID
int PIDout_PIN = 10; // change Pin for PWM Output here!

//first implementation
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=4.50, Ki=0.06, Kd=5.0; //double Kp=0.95, Ki=0.04, Kd=0.3; first values Kp=4.50, Ki=0.06, Kd=5.0; finetuned values
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,P_ON_E,DIRECT);

/*
//second implementation
//pid settings and gains
#define OUTPUT_MIN 0 
#define OUTPUT_MAX 255 //for PWM analog output
#define KP .12
#define KI .0003
#define KD 0 //preset values
double temperature, setPoint, outputVal;
AutoPID tempPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
*/
//time since start for timing and sceduling purposes
double time_passed;
unsigned long time_millis;
unsigned long time_micros;
unsigned int time_seconds;
int time_micros_overflow;



void setup() {
  Serial.begin(9600);


  //LCD Startup sequence
  //New Setup for I2C LCD:
  lcd1.init();                      // initialize the lcd 
  lcd2.init(); 
  lcd1.backlight();                 //activate the Light
  lcd2.backlight();
  // Print a message to the LCD.
  lcd1.setCursor(1,0);
  lcd2.setCursor(1,0);
  lcd1.print("Temperature");
  lcd2.print("PID-Tuning");
  lcd1.setCursor(1,1);
  lcd2.setCursor(1,1);
  lcd1.print("Control");
  lcd2.print("TestSetup");
  lcd1.blink_on();
  lcd2.blink_on();
  delay(1000);
  lcd1.blink_off();
  lcd2.blink_off();
  for (int i=0;i <= 20; i++){ 
    lcd1.setBacklight(i%2);
    lcd2.setBacklight(i%2);
    delay(100);
  }
  lcd1.backlight();                 //activate the Light
  lcd2.backlight();
  lcd1.clear();
  lcd2.clear();


  // wait for MAX chip to stabilize
  //delay(500); not needed - We already wait for the display to boot

  // Initialize Temperatures
  temp = thermocouple.readCelsius();
  settemp = (analogRead(pot_temp)/4)+35;
  Input = temp;
  Setpoint = settemp;
  lastTemp.push(temp);

  //start PID
  myPID.SetMode(AUTOMATIC);

  //Initialize PWM Output
  pinMode(PIDout_PIN, OUTPUT);
  

  // Initialize Buttons for direct control of parameters and settings:
  pinMode(b_sel, INPUT); 
  pinMode(b_inc, INPUT);
  pinMode(b_dec, INPUT);

  // Initialize Analog input for Potentiometer as direct control of temperature
  pinMode(pot_temp,INPUT);



  
  
}

void loop() {

  //time since start for timing and sceduling purposes
  time_millis = millis();
  time_micros = micros();
  time_seconds = time_millis/1000;
  time_passed = ((float)time_micros/1000);
  
  
  //Read Temperature and Settemperature
  temp = thermocouple.readCelsius();
  settemp = (analogRead(pot_temp)/3)+35;

  //Stablizing the Temperature over the last x values (determined by queueSize) with a simple FIFO 
  //put emphasis on the last value to have the smoothed tempearture more reactive
  averageTemp = averageTemp*Smoothing_Factor_avT+ temp * (1-Smoothing_Factor_avT);
  //creating an average temperature out of the last x values for smoothing
  for (int i=0;i < lastTemp.count(); i++){  //Ramping Filter
    double t = lastTemp.pop();
    averageTemp += t;
    lastTemp.push(t);
  }
  averageTemp=averageTemp/(lastTemp.count()+1);
  //adding new value for averaging and removing old one
  lastTemp.push(temp);
  if (lastTemp.count() > queueSize){
    lastTemp.pop();
  } 
  double temp_error = averageTemp - temp; //Difference for debugging

  // ramping the Settemp to get a smoothened Value
  //SmoothSettemp = settemp;
  SmoothSettemp = SmoothSettemp * Smoothing_Factor_SetT + settemp *(1-Smoothing_Factor_SetT);
  double settemp_error = SmoothSettemp-settemp; //Difference for debugging


  
  //PID
  Setpoint = SmoothSettemp;
  Input = temp;
  myPID.Compute();
  analogWrite(PIDout_PIN,Output);
  
  // Write Data to LCD
  lcd1.clear();
  lcd1.setCursor(0, 0); // go to line #1
  lcd1.print("SetTemp:");
  //lcd1.print(Setpoint); //just for debugging! 
  lcd1.print(settemp);
  lcd1.print((char)223); 
  lcd1.print("C ");
  lcd1.setCursor(0,1); // go to line #2
  lcd1.print("CurTemp:");
  lcd1.print(temp);
  lcd1.print((char)223); //Degree symbol
  lcd1.print("C ");
  
  //serial output
   Serial.print("timeFromStart=");
   Serial.print(time_passed);
   Serial.print(" ; ");
   Serial.print("Temp = "); 
   Serial.print(temp);
   Serial.print(" ; ");
   Serial.print("Settemp = "); 
   Serial.print(settemp);
   Serial.print(" ; ");
   Serial.print("Output = ");
   Serial.print(Output);
   Serial.print(" ; ");   
   Serial.print("averageTemp = ");
   Serial.print(averageTemp);
   Serial.print(" ; ");
   Serial.print("SmoothSettemp = ");
   Serial.print(SmoothSettemp);
   Serial.print(" ; ");
   Serial.print("temp_error = ");
   Serial.print(temp_error);
   Serial.print(" ; ");
   Serial.print("settemp_error = ");
   Serial.print(settemp_error);
   Serial.print(" ; ");
   Serial.print("Kp=");
   Serial.print(Kp);
   Serial.print(" ; ");
   Serial.print("Ki=");
   Serial.print(Ki);
   Serial.print(" ; ");
   Serial.print("Kd=");
   Serial.print(Kd);
   Serial.println(" ; ");
   
   
   


   //Buttons for PID finetuning:
   sel = digitalRead(b_sel);
   inc = digitalRead(b_inc);
   dec = digitalRead(b_dec);
   
   //Debug output - uncomment for sending raw values to serial
   /*
   Serial.println(sel);
   Serial.println(inc);
   Serial.println(dec);*/


   //Button Magic - Selecting the right value and +/- it
   lcd2.blink_off();

   if (sel == HIGH) { //cycle through values
    Selector++;
    }
   if (Selector >2){ //No overshoot!
    Selector=0;
   }
   if (inc == HIGH) {//++
    if (Selector ==0){
      Kp = Kp +0.01;
    }
    if (Selector ==1){
      Ki = Ki +0.01;
    }
    if (Selector ==2){
      Kd = Kd +0.01;
    }
   }
   if (dec == HIGH) {//--
    if (Selector ==0){
      Kp = Kp -0.01;
    }
    if (Selector ==1){
      Ki = Ki -0.01;
    }
    if (Selector ==2){
      Kd = Kd -0.01;
    }
   }


   // Write Data to LCD2
   lcd2.clear();
   lcd2.print("Kp=");
   lcd2.print(Kp);
   lcd2.print(";");
   lcd2.print("Ki=");
   lcd2.print(Ki);
   lcd2.print(";");
   lcd2.setCursor(0,1);
   lcd2.print("Kd=");
   lcd2.print(Kd);

   //mark current selected value ->
   if (Selector ==0){
    lcd2.setCursor(6,0);
    lcd2.blink_on();
   }
   if (Selector ==1){
    lcd2.setCursor(14,0);
    lcd2.blink_on();
   }
   if (Selector ==2){
    lcd2.setCursor(6,1);
    lcd2.blink_on();
   }

   
   delay(200);
}
