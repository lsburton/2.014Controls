//Libraries to include:
#include "Adafruit_MAX31855.h"//thermocouple amplifier library
#include <PID_v1.h>//A simple PID control library

//Define PINS
//ON/OFF Switch
int SYSTEM_STATUS = 53;

// Sodium Chlorate Components
int SC_PSLOW = 22; // 10 psi Pressure Switch
int SC_PSHIGH = 23; // 30 psi Pressure Switch
int SC_MOTOR_CURRENT = A0; // Ball-Chain Motor (Current)
int SC_MOTOR_PWM = 2; // Ball-Chain Motor (PWM)
int SC_MOTOR_DIRECTION = 24; // Ball-Chain Motor (Direction)
int SC_HEATER_CURRENT = A1; // Heater (Current)
int SC_HEATER_PWM = 3; // Heater (PWM)
int SC_HEATER_DIRECTION = 25; // Heater (Direction)
int SC_TEMP_DO = 30;
int SC_TEMP_CS = 31;
int SC_TEMP_CLK = 32;

// Fuel Feed
int FF_SERVO = 4; // Turning On/Off other Arduino
int FF_CONTROLLER = A5; // Reading flow rate of H2

// Gallium Pump
int GP_PUMP_CURRENT = A2; // Gallium Pump (Current)
int GP_PUMP_PWM = 4; // Gallium Pump (PWM)
int GP_PUMP_DIRECTION = 27; // Gallium Pump (Direction)

// Waste Management

//SODIUM CHLORATE VARIABLES AND OTHER INITIALIZATIONS
//Define PID Variables
double SC_Setpoint, SC_Temperature, SC_Output;//Setpoint, temperature reading from thermocouple, and controller output

//Specify PID and tuning parameters
PID SC_Heater_PID(&SC_Temperature, &SC_Output, &SC_Setpoint,10,5,1,DIRECT);//(Input,Output,Setpoint,Kp,Ki,Kd)

//Initialize thermocouple amp
Adafruit_MAX31855 thermocouple(SC_TEMP_CLK, SC_TEMP_CS, SC_TEMP_DO);

//variables
int SC_MIN_RUN_TEMP = 260;//Motor will begin running once heater at this temp
boolean Motor_Running = false;//Check variable, will be true once motor starts running, needed for startup and shutdown process
float SC_Heater_Temp;//Heater temperature

void setup() {
  Serial.begin(9600);
  //PINS
  //ON/OFF Switch
  pinMode(SYSTEM_STATUS, INPUT);  
  // Sodium Chlorate
  pinMode(SC_PSLOW,INPUT);
  pinMode(SC_PSHIGH,INPUT);
  pinMode(SC_MOTOR_CURRENT,INPUT);
  pinMode(SC_MOTOR_PWM,OUTPUT);
  pinMode(SC_MOTOR_DIRECTION,OUTPUT);
  pinMode(SC_HEATER_CURRENT,INPUT);
  pinMode(SC_HEATER_PWM,OUTPUT);
  pinMode(SC_HEATER_DIRECTION,OUTPUT);
  //Fuel Feel
  pinMode(FF_SERVO,OUTPUT);
  pinMode(FF_CONTROLLER,INPUT);
  //Gallium Pump
  pinMode(GP_PUMP_CURRENT,INPUT);
  pinMode(GP_PUMP_PWM,OUTPUT);
  pinMode(GP_PUMP_DIRECTION,OUTPUT);
  
  //Initialize Variables
  //Sodium Chlorate
  SC_Setpoint = 275;//Celsius
  
  //Turn on PID
  SC_Heater_PID.SetMode(AUTOMATIC);
} 

void loop(){
  int buttonState = digitalRead(SYSTEM_STATUS);
  if (buttonState == true){
    //Run the sodium chlorate Heater
    SC_Heater_Temp = run_SC_Heater(); //returns temperature of heating coil
    //Run the sodium chlorate motor
    run_SC_Motor(SC_Heater_Temp);
    //Run Fuel Feed Motor
    run_FF_MOTOR(SC_Heater_Temp);
    
  }
  else{
    //Turn off heater
    SC_Heater_Temp = turn_off_SC_Heater();
    //Run the sodium chlorate motor
    run_SC_Motor(SC_Heater_Temp);//Will shut down on own,dependent on Temp
    //Run Fuel Feed Motor
    run_FF_MOTOR(SC_Heater_Temp);//Will shut down on own, dependent on Temp
  }
    
 
 
  
}


//Running the SC Heater
float run_SC_Heater(){
  //Read SC Heater Temp and print
  SC_Temperature = thermocouple.readCelsius();
  if (isnan(SC_Temperature)){
    Serial.println("SOMETHING WRONG WITH SC THERMOCOUPLE!");
  }
  else{
    Serial.print("SC Heater (C) = ");
    Serial.println(SC_Temperature);
  }
 
  //Compute PID Output
  SC_Heater_PID.Compute();
  
  //Write to heater
  digitalWrite(SC_HEATER_DIRECTION,HIGH);
  analogWrite(SC_HEATER_PWM, SC_Output);
  int Heater_Current = analogRead(SC_HEATER_CURRENT);
  Serial.print("SC HEATER CURRENT DRAW = ");
  Serial.println(Heater_Current);
  
  return SC_Temperature;
}

//Turn off the SC Heater
float turn_off_SC_Heater(){
  //Read SC Heater Temp and print
  SC_Temperature = thermocouple.readCelsius();
  if (isnan(SC_Temperature)){
    Serial.println("SOMETHING WRONG WITH SC THERMOCOUPLE!");
  }
  else{
    Serial.print("SC Heater (C) = ");
    Serial.println(SC_Temperature);
  }
 
  //Write to heater
  digitalWrite(SC_HEATER_DIRECTION,HIGH);
  analogWrite(SC_HEATER_PWM, 0);//Turns Heater off
  int Heater_Current = analogRead(SC_HEATER_CURRENT);
  Serial.print("SC HEATER CURRENT DRAW = ");
  Serial.println(Heater_Current);
  
  return SC_Temperature;
}


//SC Motor Control
void run_SC_Motor(float Heater_Temp){
  int SC_low_pressure = digitalRead(SC_PSLOW);//LOW below 10 psi, HIGH above 10 psi
  int SC_high_pressure = digitalRead(SC_PSHIGH);//LOW below 30 psi, HIGH above--should probably change to 15-20 psi
  
  if (Heater_Temp >= SC_MIN_RUN_TEMP){
    if (SC_low_pressure == LOW && SC_high_pressure == LOW){
      //below low pressure switch
      digitalWrite(SC_MOTOR_DIRECTION,HIGH);
      analogWrite(SC_MOTOR_PWM, 255);//Should be moving as fast as dissociation can go here
      int Motor_Current = analogRead(SC_MOTOR_CURRENT);
    }
    else if (SC_low_pressure == HIGH && SC_high_pressure == LOW){
      //Between low and high pressures
      digitalWrite(SC_MOTOR_DIRECTION,HIGH);
      analogWrite(SC_MOTOR_PWM, 150);//Should be moving slower here
      int Motor_Current = analogRead(SC_MOTOR_CURRENT);
    }
    else if (SC_low_pressure == HIGH && SC_high_pressure == HIGH){
      //ABOVE low and high pressures! BAD
      //Need to keep motor moving, maybe move back and forth instead of up
      digitalWrite(SC_MOTOR_DIRECTION,HIGH);
      analogWrite(SC_MOTOR_PWM, 50);//Should be moving as slow as possible here... need to figure out that speed
      int Motor_Current = analogRead(SC_MOTOR_CURRENT);
    }
    Motor_Running = true;
  }
  else{
    //Deal with starting up and shutting off...
    //Don't want bead chain getting stuck!
    //STARTUP
    if (Motor_Running == false){
      //DO NOTHING, no motor movement
    }
    //SHUT_DOWN
    if (Motor_Running == true){
      //Keep moving slowly so chain does not get stuck until temp cools
      if (Heater_Temp >= 175){
        digitalWrite(SC_MOTOR_DIRECTION,HIGH);
        analogWrite(SC_MOTOR_PWM, 50);//Should be moving as slow as possible here... need to figure out that speed
        int Motor_Current = analogRead(SC_MOTOR_CURRENT);
      }
      //shut down motor once temp has cooled
      else{
        digitalWrite(SC_MOTOR_DIRECTION,HIGH);
        analogWrite(SC_MOTOR_PWM, 0);//Should be off
        int Motor_Current = analogRead(SC_MOTOR_CURRENT);  
      }
    }
  }  
}

//Run Fuel Feed
void run_FF_MOTOR(float temperature){
  if (temperature >= SC_MIN_RUN_TEMP){
    // turn on motor FF_SERVO
    digitalWrite(FF_SERVO,HIGH);
  }
  else{
    //turn off motor FF_SERVO
    digitalWrite(FF_SERVO,LOW);
  }
}



  
