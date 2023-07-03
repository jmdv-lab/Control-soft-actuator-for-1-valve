#include "Arduino.h"
#include <PID_v1.h>
#include "ads.h"


//Define the input pins for the sensors

#define ADS_RESET_PIN      (18)           // Bendlabs sensor Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN  (19)           // Bendlabs sensor.  

#define RUBBER_SENSOR (A0)               //  Adafruit Rubber Cord Sensor analog input
#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

//Arduino PWM Speed Control：
int E1 = 3;
int M1 = 4;
int E2 = 11;
int M2 = 12;

const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

//pressure controller
int Kp_p = 3;
int Ki_p = 10;
int Kd_p = 0.1;

double Setpoint_p ;  // the desired value 
double Input_p;  // pressure value
double Output_p; // motor speed

//angle controller
int Kp_a = 5; // initial value : 5
int Ki_a = 1; // initial value : 1
int Kd_a = 0.5; // initial value : 0.8

double Setpoint_a ;  // the desired value 
double Input_a;  // pressure value
double Output_a; // motor speed


int value=0;
    float pressure_sensorValue;
    float angle;
    int Output2;
    int Output;
    


   char command, commanda;
 int incominglen;
 float value_f, value_f_a;




// Create PID instance
PID pressure_PID(&Input_p, &Output_p, &Setpoint_p, Kp_p, Ki_p, Kd_p, DIRECT);
PID angle_PID(&Input_a, &Output_a, &Setpoint_a, Kp_a, Ki_a, Kd_a, DIRECT);

//Pressure sensor calibration factors  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)  Vout=Vs(P * 0.009 + 0.04),  Vs=5V = 1024,  P = 
 
const float SensorOffset = 4.44;  //pressure sensor offset
const float SensorGain = 0.109;   // pressure sensor proportional relation

//BendSensor functions for dataprocessing

void ads_data_callback(float * sample);                          // Bendlabs sensor information callback
void deadzone_filter(float * sample);                            // Bendlabs sensor Deadzone filter
void signal_filter(float * sample);                              // Bendlabs signal filter  
void parse_com_port(void);                                       // I2C communication decoding

/* Not used in polled mode. Stub function necessary for library compilation */
void ads_data_callback(float * sample, uint8_t sample_type)
{
  
}


//Motor shield functions
void motor_1_on(int motorspeed);   // Pump 1 activation motor speed range between 0 - 255   --> 0 - 5 Volts
void motor_2_on(int motorspeed);   // Pump 2 activation motor speed range between 0 - 255   --> 0 - 5 Volts
void motor_1_off(void);            // Pump 1 deactivation  (motor speed == 0)
void motor_2_off(void);            // Pump 2 deactivation  (motor speed == 0)

void valve_1_on(void);             // Valve 1 activated -->  5V
void valve_2_on(void);             // Valve 2 activated -->  5V
void valve_1_off(void);            // Valve 1 deactivated -->  0V
void valve_2_off(void);            // Valve 2 deactivated -->  0V



void setup()
{
    
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);
    pinMode(PRESSURE_SENSOR, INPUT);  // Defining sensor inputs for ADC (Analog digital converter)
    
    Serial.begin(115200);             // Starting Serial communication with computer baudrate 115200 bps

    //Hardcode the pressure and angle value
    Setpoint_a = 15;
    Setpoint_p = 15;
   
    //Turn the PID on
    pressure_PID.SetMode(AUTOMATIC);
    angle_PID.SetMode(AUTOMATIC);
    //Adjust PID values
    pressure_PID.SetTunings(Kp_p, Ki_p, Kd_p); 
    pressure_PID.SetOutputLimits(-255,255);  

    angle_PID.SetTunings(Kp_a, Ki_a, Kd_a); 
    angle_PID.SetOutputLimits(-255,255);  


// Initialization routine for bendlabs sensor

  Serial.println("Initializing One Axis sensor");
  ads_init_t init;                                // One Axis ADS initialization structure
  init.sps = ADS_100_HZ;                          // Set sample rate to 100 Hz (Interrupt mode)
  init.ads_sample_callback = &ads_data_callback;  // Provide callback for new data
  init.reset_pin = ADS_RESET_PIN;                 // Pin connected to ADS reset line
  init.datardy_pin = ADS_INTERRUPT_PIN;           // Pin connected to ADS data ready interrupt
  init.addr = 0;                                  // Update value if non default I2C address is assinged to sensor
  int ret_val = ads_init(&init);                 // Initialize ADS hardware abstraction layer, and set the sample rate
  if(ret_val != ADS_OK)
  {
    Serial.print("One Axis ADS initialization failed with reason: ");
    Serial.println(ret_val);
  }
  else
  {
    Serial.println("One Axis ADS initialization succeeded...");
  }
  // Enable stretch measurements
  ads_stretch_en(true);
  // Start reading data in polled mode
  ads_polled(true);
  // Wait for first sample
  delay(10);


}

void loop()
{


  //Defining varialbes for bendlabs sensor data processing

  static float sample[2];
  uint8_t data_type;
  float Setpoint_f;
  int p_limit;
  
  

    //Reading pressure sensor
    pressure_sensorValue = (analogRead(PRESSURE_SENSOR)*SensorGain-SensorOffset); //Do maths for calibration

    //Reading bendsensor

     // Read data from the one axis ads sensor
  int ret_val = ads_read_polled(sample, &data_type);

  // Check if read was successfull
  if(ret_val == ADS_OK)
  {
    if(data_type == ADS_SAMPLE)
    {
      // Low pass IIR filter
      signal_filter(sample);

      // Deadzone filter
      deadzone_filter(sample);
    }
  }
  
    angle=1.1591*sample[0]-11.8166;

          //PWM Speed Control


       // Check for received hot keys on the com port
  if(Serial.available())
  {
 commanda=command;
 value_f_a=value_f;
    //parse_com_port();
  //double incomingValue = Serial.parseFloat();
 String incomingValue = Serial.readStringUntil('\n');
 command=incomingValue[0];
 incominglen=incomingValue.length();
 String value =incomingValue.substring(1,incominglen);
 value_f=value.toFloat();

 
  //Serial.print("I received: ");
  //Serial.println(value_f);


  if (command == 'c')
 {
  char calibrator=incomingValue[1];
  parse_com_port(calibrator);
  }

  }
    //
  if (command == 'a')
  {
  Setpoint_a = value_f;
  Setpoint_f=Setpoint_a;
  Output=Output_a; 
    }

  else if (command == 'p')
  {
  Setpoint_p = value_f;
  Setpoint_f=Setpoint_p;
  Output=Output_p; 
    }

 else if (command == 'm')
  {
  Setpoint_p = 22;
  Setpoint_f=0;
  Output=value_f; 
    }
 else if (command == 'r')
  {
    Serial.print(1.1591*sample[0]-11.8166);    // Angle data
    Serial.print(","); 
    Serial.print(sample[0]);    // pressure data in kpa
    Serial.print(",");
    Serial.print(value_f);
    Serial.print(",");
    Serial.print(Output);
    Serial.println(",");
  command = commanda;
  Setpoint_a = value_f;
  Setpoint_f=Setpoint_a;
  Output=Output_a; 
    }

Input_a = angle;
Input_p = pressure_sensorValue;
pressure_PID.Compute();  //
angle_PID.Compute();  //

  if (command == 'a')
  {
  Output=Output_a;
  p_limit=30; 
    }

  else if (command == 'p')
  {
  Output=Output_p;
  p_limit=30; 
    }

  else if (command == 'm')
  {
  Setpoint_p = 22;
  Setpoint_f=0;
  Output=value_f; 
  p_limit=27;
    }

   
    

 if (pressure_sensorValue <= p_limit and angle<=185)
    {
        if (Output >= 0)
        {
          Output2 = map(Output, 0, 255, 15, 250); 
          motor_1_on(Output2);   //PWM Speed Control   value
          valve_2_on();

          int errores = Input_a - Setpoint_a; 
          if (errores<0 and Output2>=100)
          {
            valve_2_off(); 
            delay(0.5);
            valve_2_on();
          }
          else
          {
            valve_2_on();
          }

          }
          
        else 
        {
          //Output2 = map(-Output, 0, 255, 200, 15); 
          //motor_1_on(Output2);   //PWM Speed Control   value
          motor_1_off();   //PWM Speed Control   value
          valve_2_off(); 
         }
    }
    else 
    {
       //motor_1_on(45);   //PWM Speed Control   value
       motor_1_off();   //PWM Speed Control   value
       valve_2_off();
    }


   Serial.print(sample[0]);    // Angle data
   Serial.print(","); 
   Serial.print(p_limit);    // Angle data
   Serial.print(","); 
   Serial.print(1.1591*sample[0]-11.8166);    // Angle data
   Serial.print(","); 
   Serial.print(Setpoint_f);
   Serial.print(",");
   Serial.println(Output);
    
    delay(10);
    
}


/* Function parses received characters from the COM port for commands */
void parse_com_port(char key)
{
  //char key = Serial.read();

  switch(key)
  {
    case '0':
      // Take first calibration point at zero degrees
      ads_calibrate(ADS_CALIBRATE_FIRST, 0);
      Serial.println("0 calibrated");
      break;
    case '9':
      // Take second calibration point at ninety degrees
      ads_calibrate(ADS_CALIBRATE_SECOND, 90);
      Serial.println("90 calibrated");
      break;
    case 'c':
      // Restore factory calibration coefficients
      ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
      break;
    case 'r':
      // Start sampling in interrupt mode
      ads_run(true);
      break;
    case 's':
      // Place ADS in suspend mode
      ads_run(false);
      break;
    case 'f':
      // Set ADS sample rate to 200 Hz (interrupt mode)
      ads_set_sample_rate(ADS_200_HZ);
      break;
    case 'u':
      // Set ADS sample to rate to 10 Hz (interrupt mode)
      ads_set_sample_rate(ADS_10_HZ);
      break;
    case 'n':
      // Set ADS sample rate to 100 Hz (interrupt mode)
      ads_set_sample_rate(ADS_100_HZ);
      break;
    case 'b':
      // Calibrate the zero millimeter linear displacement
      ads_calibrate(ADS_CALIBRATE_STRETCH_ZERO, 0);
      break;
    case 'e':
      // Calibrate the 30 millimeter linear displacement (stretch), Make certain the sensor is at 0 degrees angular displacement (flat)
      ads_calibrate(ADS_CALIBRATE_STRETCH_SECOND, 30);
      break;
    default:
      break;
  }
}

/* 
 *  Second order Infinite impulse response low pass filter. Sample freqency 100 Hz.
 *  Cutoff freqency 20 Hz. 
 */
void signal_filter(float * sample)
{
    static float filter_samples[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[i][5] = filter_samples[i][4];
      filter_samples[i][4] = filter_samples[i][3];
      filter_samples[i][3] = (float)sample[i];
      filter_samples[i][2] = filter_samples[i][1];
      filter_samples[i][1] = filter_samples[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
        0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

      sample[i] = filter_samples[i][0];
    }
}

/* 
 *  If the current sample is less that 0.5 degrees different from the previous sample
 *  the function returns the previous sample. Removes jitter from the signal. 
 */
void deadzone_filter(float * sample)
{
  static float prev_sample[2];
  float dead_zone = 0.75f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}

// Motor control functions 

void motor_1_on(int motorspeed)
{
  analogWrite(E1, motorspeed);   //PWM Speed Control   value
  digitalWrite(M1,HIGH);
  }

void motor_1_off(void)
{
  analogWrite(E1, 0);   //PWM Speed Control   value
  digitalWrite(M1,HIGH);
  }

  void valve_1_on(void)
{
  analogWrite(E2, 255);   //PWM Speed Control   value
  digitalWrite(M2,HIGH);
  }

void valve_1_off(void)
{
  analogWrite(E2, 0);   //PWM Speed Control   value
  digitalWrite(M2,HIGH);
  }

void motor_2_on(int motorspeed)
{
  analogWrite(E3, motorspeed);   //PWM Speed Control   value
  digitalWrite(M3,HIGH);
  }

void motor_2_off(void)
{
  analogWrite(E3, 0);   //PWM Speed Control   value
  digitalWrite(M3,HIGH);
  }

  void valve_2_on(void)
{
  analogWrite(E4, 255);   //PWM Speed Control   value
  digitalWrite(M4,HIGH);
  }

void valve_2_off(void)
{
  analogWrite(E4, 0);   //PWM Speed Control   value
  digitalWrite(M4,HIGH);
  }
