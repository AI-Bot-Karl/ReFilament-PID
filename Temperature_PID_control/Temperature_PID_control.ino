

#include <max6675.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <PID_v1.h>
#include <QueueArray.h>


//THERMOCOUPLE
int thermoDO = 2;
int thermoCS = 3;
int thermoCLK = 4;
//Create Reading for MAX6675
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

double temp;  //holds actual Temp  
double settemp;  //holds Settemp  
QueueArray <double> lastTemp;  //holds last few temp values to get an average
int queueSize = 5;  //sets amount of values for average

//DISPLAY

LiquidCrystal_I2C lcd(0x26,16,2);


//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=0.95, Ki=0.04, Kd=0.3;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,P_ON_E,DIRECT);

int PIDout = 5;


//IO for PID finetuning:
LiquidCrystal_I2C lcd2(0x27,16,2);

bool AState1 = false, AState2 = false, AState3 = false; // Variables to hold the state of the Analog input

int Selector = 0; // for choosing wich value to change

void setup() {
  Serial.begin(9600);
  
  //New Setup for I2C LCD:
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Temperature");
  lcd.setCursor(1,1);
  lcd.print("Control");
  lcd.blink_on();
  delay(2000);
  lcd.blink_off();
  lcd.clear();

  // wait for MAX chip to stabilize
  delay(500);
  
  temp = thermocouple.readCelsius();
  settemp = (analogRead(A0)/4)+35;
  
  Input = temp;
  Setpoint = 35;
  
  myPID.SetMode(AUTOMATIC);
  
  lastTemp.push(temp);

//IO for PID finetuning:

  lcd2.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd2.backlight();
  lcd2.setCursor(1,0);
  lcd2.print("PID-Tuning");
  lcd2.setCursor(1,1);
  lcd2.print("TestSetup");
  lcd2.blink_on();
  delay(2000);
  lcd2.blink_off();
  lcd2.clear();

  pinMode(A1, INPUT); //Initialize the analog input for the buttons
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);


  
  
}

void loop() {

  
  temp = thermocouple.readCelsius();
  settemp = (analogRead(A0)/3)+35;
  
  //adding new value for averaging and removing old one
  lastTemp.push(temp);
  if (lastTemp.count() > queueSize)
  {
    lastTemp.pop();
  }
  
  //creating an average temperature out of the last x values for smoothing
  double averageTemp;   //New Temp Variable to be used with the PID
  for (int i=0;i < lastTemp.count(); i++){  //Ramping Filter
    double t = lastTemp.pop();
    averageTemp += t;
    lastTemp.push(t);
  }
  /*double Smoothing_Factor_avT = 0.9; // Smoothing Filter
  averageTemp = averageTemp*Smoothing_Factor_avT+ temp * (1-Smoothing_Factor_avT);
  averageTemp = averageTemp/lastTemp.count();*/


  // ramping the Settemp to get a smoothened heat up curve
  int SmoothSettemp; //New Settemp to be used with the PID
  SmoothSettemp = settemp;
  /*double Smoothing_Factor_SetT = 0.9; //Factor for linear Smoothing
  SmoothSettemp = SmoothSettemp * Smoothing_Factor_SetT + settemp *(1-Smoothing_Factor_SetT);
  double Ramping = SmoothSettemp-settemp * */
  
    
  
  
  
  //PID
  Setpoint = SmoothSettemp;
  Input = temp;
  myPID.Compute();
  analogWrite(PIDout,Output);
  
  // LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SetTemp:");
  lcd.print(Setpoint);
  lcd.print((char)223);
  lcd.print("C ");
  

  // go to line #1
  lcd.setCursor(0,1);
  lcd.print("CurTemp:");
  lcd.print(temp);
  lcd.print((char)223);
  lcd.print("C ");
  
  //serial output
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
   Serial.print("Kp=");
   Serial.print(Kp);
   Serial.print(" ; ");
   Serial.print("Ki=");
   Serial.print(Ki);
   Serial.print(" ; ");
   Serial.print("Kd=");
   Serial.print(Kd);
   Serial.println(" ; ");
   
   
   


//IO for PID finetuning:
int analog1 = analogRead(A1); //reading the analog only ONE TIME per Cycle
int analog2 = analogRead(A2);
int analog3 = analogRead(A3);

AState1 = analog1 > 100 ? HIGH : LOW; // A/D
AState2 = analog2 > 100 ? HIGH : LOW;
AState3 = analog3 > 100 ? HIGH : LOW;
 //Debug output:
Serial.println(AState1);
Serial.println(AState2);
Serial.println(AState3);
Serial.println(analog1);
Serial.println(analog2);
Serial.println(analog3);

//Button Magic - Selecting the right value and +/- it
lcd2.blink_off();

if (AState1 == HIGH) { //cycle through values
  Selector++;
  
}
if (Selector >2){ //No overshoot!
  Selector=0;
}



if (AState2 == HIGH) {//++
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
if (AState3 == HIGH) {//--
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


// write the stuff to the display
lcd2.clear();
//double Kp=0.95, Ki=0.04, Kd=0.3;
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
