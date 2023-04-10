/*
  Author: Jishnu syam
  Description: This code is an Arduino sketch that implements a control algorithm 
  for a solar power management system.The system consists of a solar panel (PV),a battery,
  and a load, and includes a speaker connected across Vds of the load converter to generate ultrasonic sound 
  Date: April 8, 2023
  Contact: kjs13@iitbbs.ac.in 
  
*/



#include <PWM.h>           // Include PWM library for controlling PWM signals

//defining pins

#define LOAD_PWM_PIN 3            // PWM output pin for controlling the load
#define PV_PWM_PIN 10             // Pin number for PV Buck PWM output
#define PV_V A0                   // Analog feedback pin for reading PV voltage
#define PV_I A1                   // Analog feedback pin for reading PV Current
#define Bat_V A2                  // Analog feedback pin for reading Battery Voltage
#define Load_V A3                 // Analog feedback pin for reading load voltage
 
////////////////// const declarations for Load ///////////////////////////////

const int Load_ref_V = 20;         // Load reference voltage
const unsigned long LSF = 12000;   // sweep start frequency LSF ---> (Load start frequency) 
const unsigned long LEF = 60000;   // sweep End frequency LEF ---> (Load END frequency)
long LF = 28000;                   // Initial frequency  LF ---> (Load  frequency)
int FSD = 50;                      // Delay between frequency sweeps  FSD ---> (frequency sweep delay)
int FSI = 1000;                     // Increment value for changing frequency  LF ---> (frequency sweep increment)


float Load_dutyCycle;              // Duty cycle of the PWM signal        (Note: in this code dutycycles are mapped 0-100% to 0-254 )
float Load_error;                  // Error between setpoint and feedback voltage
float integralError = 0.0;         // Integral error for PID controller
float derivativeError = 0.0;       // Derivative error for PID controller
float previousError = 0.0;         // Previous error for calculating derivative error


// defining PID controller coefficients for Load voltage control

#define kp 5.0   // Proportional coefficient for PID controller
#define ki 0.8   // Integral coefficient for PID controller
#define kd 0.01  // Derivative coefficient for PID controller



 ////////////////// const declarations for PV ///////////////////////////////
 
const unsigned long PV_frequency = 60000;  //PWM switching frequency for Buck converter
const unsigned long SampleTime = 50;      // Sampling time in milliseconds for MPPT
unsigned long LastSampleTime = 0;         // Variable to store the last sample time


// Battery voltage thresholds for different charging modes
const float bulk_voltage_min = 13.000; 
const float bulk_voltage_max = 14.500; 
const float absorption_voltage = 14.700; 
const float float_voltage_min = 13.200; 
const float float_voltage = 13.400; 
const float float_voltage_max = 13.000; 
const float bat_low_voltage = 11.2;        
const float HYSTERESIS = 0.2;           // Hysteresis value for low battery cutoff
bool load_switch = 0;                   // to turn  boost converter on/off



// Maximum and minimum current limits for different charging modes in mA
const float float_max_current = 120;  
const float absorption_max_current = 2000;
const float absorption_min_current = 200;

// Variables to store PV (solar panel) parameters
float PV_voltage ;
float PV_current ;  
float PV_Power ;
float PV_voltage_old;
float PV_power_old;

// Battery voltage and power change rate variables
float bat_voltage; 
float dP ;
float dV ;
int d_old = 200;           // Variable to store previous duty cycle
int delta_d = 3;           // Change in duty cycle for each iteration
float pv_DutyCycle = 200;  // Variable to control duty cycle of PWM signal initialized with 78% dutycycle


//Given values to each battery mode of operation
byte BULK = 0;        
byte ABSORPTION = 1;
byte FLOAT = 2;
byte mode = 0;        //We start with mode 0 BULK





void setup() { 
  
  Serial.begin(115200);               // Initialize serial communication at a baud rate of 115200
  analogReference(EXTERNAL);          // Set the analog reference voltage to external (3.3V in this case)
  pinMode(A0, INPUT);                 // Set pin A0 as an input (PV voltage)
  pinMode(A1, INPUT);                 // Set pin A1 as an input (PV current)
  pinMode(A2, INPUT);                 // Set pin A2 as an input (Battery voltage)
  pinMode(A3, INPUT);                 // Set pin A3 as an input (Load voltage)
  InitTimersSafe();                   // Initialize timers for PWM 
  pinMode(LOAD_PWM_PIN, OUTPUT);      // Set PWM output pin as OUTPUT
  pinMode(PV_PWM_PIN, OUTPUT);        // Set PWM output pin as OUTPUT
  
}

void loop() {


////////////////////// Load/////////////////////////


  if ( bat_voltage <= (bat_low_voltage - HYSTERESIS)) {          // Battery voltage is below low threshold, turn off boost converter    
    load_switch = 0;
    Load_dutyCycle = 0;
  } 
  
  else if ( bat_voltage >= (bat_low_voltage + HYSTERESIS)) {    // Battery voltage is above high threshold, turn on boost converter 
     load_switch = 1;
     if (millis() - ((FSD * abs(millis()) / FSD)) == 0){        // Frequency sweep logic: update frequency based on delay and increment value
          LF = LF + FSI;
          if (LF <= LSF || LF >= LEF){
          FSI = -FSI;
          }
     }

     Load_error = Load_ref_V - (analogRead(Load_V)*0.036);                        // Calculate error between setpoint and feedback voltage
     integralError += Load_error;                                                  // Update integral error
     derivativeError = Load_error - previousError;                                 // Calculate derivative error
     Load_dutyCycle = kp * Load_error + ki * integralError + kd * derivativeError; // Calculate duty cycle using PID formula
     previousError = Load_error;                                                   // Update previous error for next iteration
     Load_dutyCycle = constrain(Load_dutyCycle,0,200);
  }

////////////////// for PV /////////////////////////////

  unsigned long now = millis();
 
  if ((now - LastSampleTime) >= SampleTime) {
      
     PV_voltage  = ((analogRead(PV_V)*36.06)/1000);      // Read analog input A0 as PV voltage and convert to voltage
     PV_current  =((analogRead(PV_I)*35.445)/10);        // Read analog input A1 as PV current and convert to mA
     PV_Power    = PV_current *PV_voltage;               // Calculate PV power by multiplying PV voltage and current
     bat_voltage = ((analogRead(Bat_V)*15.36)/1000);     // Read analog input A2 as battery voltage and convert to voltage
     dV = PV_voltage-PV_voltage_old;                     // Calculate change in PV voltage
     dP = PV_Power-PV_power_old;                         // Calculate change in PV power
     
    
    ///////////////////////////FLOAT///////////////////////////
  ///////////////////////////////////////////////////////////
  
  if(mode == FLOAT){
    if(bat_voltage < float_voltage_min){
      mode = BULK;    // Switch to BULK mode if battery voltage is below minimum for FLOAT mode
    }
    
    else{
      if(PV_current > float_max_current){    // If PV current exceeds max current for FLOAT mode, switch to BULK mode
        mode = BULK;
      }//End if > 
  
      else{
        if(bat_voltage > float_voltage){
          pv_DutyCycle= pv_DutyCycle- delta_d;     // Decrease duty cycle if battery voltage is higher than FLOAT voltage
          pv_DutyCycle = constrain(pv_DutyCycle,10,254);
        } 
        
        else {
          pv_DutyCycle= pv_DutyCycle+ delta_d;  // Increase duty cycle if battery voltage is lower than FLOAT voltage
          pv_DutyCycle = constrain(pv_DutyCycle,10,254);
        }        
      }//End else > float_max_current
      
      
    }
  d_old = pv_DutyCycle;
  }//END of mode == FLOAT



  //Bulk/Absorption
  
  else{
    if(bat_voltage < bulk_voltage_min){
      mode = BULK;    // Switch to BULK mode if battery voltage is below minimum for BULK mode
    }
    else if(bat_voltage > bulk_voltage_max){
      mode = ABSORPTION;  // Switch to ABSORPTION mode if battery voltage is above maximum for BULK mode
    }

    
  
    ////////////////////////////BULK///////////////////////////
    ///////////////////////////////////////////////////////////
    
    if(mode == BULK){

      // Maintain MPPT

         if (dP<0){       // If power change is negative
           if (dV<0){     // If voltage change is negative
              pv_DutyCycle= d_old - delta_d;   
              pv_DutyCycle = constrain(pv_DutyCycle,10,254);
              
           }
           else{
              pv_DutyCycle= d_old + delta_d;   
              pv_DutyCycle = constrain(pv_DutyCycle,10,254);
              
           }
         }
         else{     
           if (dV<=0){           // If voltage change is not positive
             pv_DutyCycle= d_old + delta_d; 
             pv_DutyCycle = constrain(pv_DutyCycle,10,254);
             
           }
           else{
             pv_DutyCycle= d_old - delta_d; 
             pv_DutyCycle = constrain(pv_DutyCycle,10,254);
             
           }    
        
         }
           
      
    }//End of mode == BULK
  
  
  
    /////////////////////////ABSORPTION/////////////////////////
    ///////////////////////////////////////////////////////////

    
    
    if(mode == ABSORPTION){
      if(PV_current > absorption_max_current){    //If we exceed max current value, we reduce duty cycle
        pv_DutyCycle= pv_DutyCycle- delta_d;
        pv_DutyCycle = constrain(pv_DutyCycle,10,254);
      }//End if > absorption_max_current
  
      else{
        if(bat_voltage < absorption_voltage){    // Check if battery voltage is less than absorption voltage
          pv_DutyCycle= pv_DutyCycle+ delta_d;
          pv_DutyCycle = constrain(pv_DutyCycle,10,254);
        }
        
        else {
          pv_DutyCycle= pv_DutyCycle- delta_d;
          pv_DutyCycle = constrain(pv_DutyCycle,10,254);
        }
        
  if(PV_current < absorption_min_current){    // Check if PV current falls below minimum absorption current
          mode = FLOAT;
        }
      }//End else > absorption_max_current
      
      
    }// End of mode == absorption_max_current
    
  }//END of else mode == FLOAT

     d_old = pv_DutyCycle;          // Update old duty cycle with current duty cycle
     PV_voltage_old = PV_voltage ; // Update old PV voltage with current PV voltage
     PV_power_old = PV_Power;      // Update old PV power with current PV power
     LastSampleTime = now;         // Update last sample time with current time

  }


    bool success = SetPinFrequencySafe(LOAD_PWM_PIN,LF);            
    pwmWrite(LOAD_PWM_PIN, Load_dutyCycle);                           // Write duty cycle & to Boost converter
    bool success1 = SetPinFrequencySafe(PV_PWM_PIN, PV_frequency);  
    pwmWrite(PV_PWM_PIN, pv_DutyCycle);                              // Write duty cycle to PV PWM output pin


// printing the PV,Battery& load parameters for debugging 

     Serial.print("  PV_V = ");Serial.print(PV_voltage);Serial.print("V");
     Serial.print("  PV_I = ");Serial.print(PV_current);Serial.print("mA");
     Serial.print("  PV_P = ");Serial.print(PV_Power );Serial.print("mW");
     Serial.print("  pv_DutyCycle = ");Serial.print((pv_DutyCycle*100)/254);Serial.print("%");
     Serial.print("  Bat_V= ");Serial.print(bat_voltage);Serial.print("V");
     Serial.print("  Load_V= ");Serial.print(analogRead(Load_V) * 0.036);Serial.print("V");
     Serial.print("  Load_frequency= ");Serial.print(LF/1000);Serial.print("KHz");
     Serial.print(" Load_DutyCycle = ");Serial.print((Load_dutyCycle*100)/254);Serial.print("%");
     Serial.print(" Load ");Serial.print(load_switch);
     
     if (mode == 0){  
     Serial.print("  Mode = ");Serial.println("Bulk");
     }
     
     else if (mode == 1){
     Serial.print("  Mode = ");Serial.println("Absorbtion");
     }
     
     else if (mode == 2){
     Serial.print("  Mode = ");Serial.println("Float");
     }
     
  

}
