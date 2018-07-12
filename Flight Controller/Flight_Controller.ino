#include <HMC5883L.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // interrupt pin for IMU

#define MAX_SIGNAL 1200
#define MIN_SIGNAL 1000
#define START_SIGNAL 1100
#define ESC_CCW1 3
#define ESC_CCW2 5
#define ESC_CW1 6
#define ESC_CW2 11

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector from MPU6050
float ypr_angles[3];    // [yaw, pitch, roll]   pitch/roll 0-360 angles from MPU6050 and yaw from HMC5883L 

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

//pid constants and limits
float pid_p_gain_pitchroll = 0.0;               //Gain setting for the pitch & roll P-controller
float pid_i_gain_pitchroll = 0.00;              //Gain setting for the pitch & roll I-controller
float pid_d_gain_pitchroll = 20.0;              //Gain setting for the pitch & roll D-controller
int pid_max_pitchroll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller.
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller.
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

//ypr errors 
float yaw_correction, pitch_correction, roll_correction;
float current_yaw_error, current_pitch_error, current_roll_error;
float previous_yaw_error, previous_pitch_error, previous_roll_error; 
float pid_i_error_yaw_total, pid_i_error_pitch_total, pid_i_error_roll_total; 

//
int ccw1_signal, ccw2_signal, cw1_signal, cw2_signal;
float yaw_set_point, pitch_set_point, roll_set_point;
short throttle = 1200;

MPU6050 mpu;
HMC5883L compass;

Servo esc_ccw1;
Servo esc_ccw2;
Servo esc_cw1;
Servo esc_cw2;


void setup() {
  Serial.begin(115200);
  
  //attach all escs
  esc_ccw1.attach(ESC_CCW1);
  esc_ccw2.attach(ESC_CCW2);
  esc_cw1.attach(ESC_CW1);
  esc_cw2.attach(ESC_CW2);

  //set all moters in 00% speed
  esc_ccw1.writeMicroseconds(MIN_SIGNAL);
  esc_ccw2.writeMicroseconds(MIN_SIGNAL);
  esc_cw1.writeMicroseconds(MIN_SIGNAL);
  esc_cw2.writeMicroseconds(MIN_SIGNAL);

  mpu6050_init();
  hmc5883l_init();

  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                     //Set PCINT4 (digital input 12)to trigger an interrupt on state change.

  wait_for_stable_sensor_data();
  yaw_read();
  
  yaw_set_point = ypr_angles[0];
  pitch_set_point = 0;
  roll_set_point = 0;
}

void loop() {
  //update current yaw pitch roll variables
  ypr_read();
  yaw_read();
  
  pid_calculation();
  
  ccw1_signal = throttle - roll_correction - pitch_correction - yaw_correction;
  ccw2_signal = throttle + roll_correction + pitch_correction - yaw_correction;
  cw1_signal = throttle + roll_correction - pitch_correction + yaw_correction;
  cw2_signal = throttle - roll_correction + pitch_correction + yaw_correction;

  if(ccw1_signal < 1080) ccw1_signal = 1080;
  if(ccw2_signal < 1080) ccw2_signal = 1080;
  if(cw1_signal < 1080) cw1_signal = 1080;
  if(cw2_signal < 1080) cw2_signal = 1080;

  if(ccw1_signal > 1250) ccw1_signal = 1250;
  if(ccw2_signal > 1250) ccw2_signal = 1250;
  if(cw1_signal > 1250) cw1_signal = 1250;
  if(cw2_signal > 1250) cw2_signal = 1250;

  /*esc_ccw1.writeMicroseconds(ccw1_signal);
  esc_ccw2.writeMicroseconds(ccw2_signal);
  esc_cw1.writeMicroseconds(cw1_signal);
  esc_cw2.writeMicroseconds(cw2_signal);*/
  
  /*Serial.print(ccw2_signal);
  Serial.print("\t");
  Serial.println(cw1_signal);
  Serial.print(cw2_signal);
  Serial.print("\t");
  Serial.println(ccw1_signal);
  Serial.println();*/
  
  Serial.print("ypr\t");
  Serial.print(ypr_angles[0]);
  Serial.print("\t");
  Serial.print(ypr_angles[1]);
  Serial.print("\t");
  Serial.println(ypr_angles[2]);

  /*Serial.print(ypr_angles[0]);
  Serial.print("\t");
  Serial.println(yaw_set_point);*/
}


void wait_for_stable_sensor_data(){
  int start;
  for(start = 0; start < 1000; start++){
    ypr_read();
  }
}


void pid_calculation(){
  //yaw error calculation
  current_yaw_error = yaw_set_point - ypr_angles[0];
  pid_i_error_yaw_total += current_yaw_error * pid_i_gain_yaw;
  if(pid_i_error_yaw_total > pid_max_yaw){
    pid_i_error_yaw_total = pid_max_yaw;
  }else if(pid_i_error_yaw_total < pid_max_yaw*(-1)){
    pid_i_error_yaw_total = pid_max_yaw*(-1);
  }

  yaw_correction = pid_i_error_yaw_total + current_yaw_error * pid_p_gain_yaw + pid_d_gain_yaw * (current_yaw_error - previous_yaw_error);
  if(yaw_correction > pid_max_yaw){
    yaw_correction = pid_max_yaw;
  }else if(yaw_correction < pid_max_yaw*(-1)){
    yaw_correction = pid_max_yaw*(-1);
  }
  
  previous_yaw_error = current_yaw_error;
  
  
  //pitch error calculation
  current_pitch_error = pitch_set_point - ypr_angles[1];
  pid_i_error_pitch_total += current_pitch_error * pid_i_gain_pitchroll;
  if(pid_i_error_pitch_total > pid_max_pitchroll){
    pid_i_error_pitch_total = pid_max_pitchroll;
  }else if(pid_i_error_pitch_total < pid_max_pitchroll*(-1)){
    pid_i_error_pitch_total = pid_max_pitchroll*(-1);
  }

  pitch_correction = pid_i_error_pitch_total + current_pitch_error * pid_p_gain_pitchroll + pid_d_gain_pitchroll * (current_pitch_error - previous_pitch_error);
  if(pitch_correction > pid_max_pitchroll){
    pitch_correction = pid_max_pitchroll;
  }else if(pitch_correction < pid_max_pitchroll*(-1)){
    pitch_correction = pid_max_pitchroll*(-1);
  }
  
  previous_pitch_error = current_pitch_error;

  
  //roll error calculation
  current_roll_error = roll_set_point - ypr_angles[2];
  pid_i_error_roll_total += current_roll_error * pid_i_gain_pitchroll;
  if(pid_i_error_roll_total > pid_max_pitchroll){
    pid_i_error_roll_total = pid_max_pitchroll;
  }else if(pid_i_error_roll_total < pid_max_pitchroll*(-1)){
    pid_i_error_roll_total = pid_max_pitchroll*(-1);
  }

  roll_correction = pid_i_error_roll_total + current_roll_error * pid_p_gain_pitchroll + pid_d_gain_pitchroll * (current_roll_error - previous_roll_error);
  if(roll_correction > pid_max_pitchroll){
    roll_correction = pid_max_pitchroll;
  }else if(roll_correction < pid_max_pitchroll*(-1)){
    roll_correction = pid_max_pitchroll*(-1);
  }
  
  previous_roll_error = current_roll_error;
}

