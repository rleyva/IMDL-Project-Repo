/*-----------------------------------------------
 *  Gombot the Gomboc
 *  Author: Ralph F. Leyva
 *  Filename: Gombot.ino
 *  Revision: 1.0.0
 *  Date: 4/4/15
 *----------------------------------------------*/
 
#include <Wire.h>
#include <Servo.h>

#include <PID_v1.h>
#include "I2Cdev.h"

#include <DistanceSRF04.h>
#include <DualVNH5019MotorShield.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"

#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;
DualVNH5019MotorShield md;

// Distance Sensor values 
#define DIST_THRESHHOLD 10
DistanceSRF04 left_distance_sensor, center_distance_sensor, right_distance_sensor;
int left_distance, center_distance, right_distance;
int distance_lookup_table[3] = {0,0,0};
int SRF_poll_check = 0;
int SRF_current_time, SRF_previous_time;

// -- DMP Variables -- //
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// -- Orientation/motion Variables -- //
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// -- PID Variables -- //
#define PID_MAX 100
#define PID_SAMPLE_TIME 20
double pid_setpoint, pid_input, pid_output;
PID gombot_PID(&pid_input, &pid_output, &pid_setpoint, 4000, 1700, 100, DIRECT);

// -- Motor Control States --//
#define FORWARD_RAD_OFFSET
#define REVERSE_RAD_OFFSET

// Motor Portion of the code
// Make this portion of the code floats
double left_motor_control = -1;
double right_motor_control = -1;


void check_obstacles(int left_distance, int center_distance, int right_distance){
  // NASTY! Reimplement as a LUT
  //digitalWrite(53, HIGH);
  if(left_distance >= DIST_THRESHHOLD && center_distance >= DIST_THRESHHOLD && right_distance >= DIST_THRESHHOLD){
          digitalWrite(53, LOW);  digitalWrite(49, LOW);  digitalWrite(51, LOW);
      distance_lookup_table[0] = 0; distance_lookup_table[1] = 0; distance_lookup_table[2] = 0;
  }
  else if(left_distance <= DIST_THRESHHOLD && center_distance > DIST_THRESHHOLD && right_distance > DIST_THRESHHOLD){
      digitalWrite(53, HIGH);  digitalWrite(49, LOW);  digitalWrite(51, LOW);
      distance_lookup_table[0] = 1; distance_lookup_table[1] = 0; distance_lookup_table[2] = 0;
  }
  else if(left_distance > DIST_THRESHHOLD && center_distance <= DIST_THRESHHOLD && right_distance > DIST_THRESHHOLD){
      digitalWrite(53, LOW);  digitalWrite(49, HIGH);  digitalWrite(51, LOW);
      distance_lookup_table[0] = 0; distance_lookup_table[1] = 1; distance_lookup_table[2] = 0;
  }
  else if(left_distance > DIST_THRESHHOLD && center_distance > DIST_THRESHHOLD && right_distance < DIST_THRESHHOLD){
      digitalWrite(53, LOW);  digitalWrite(49, LOW);  digitalWrite(51, HIGH);
      distance_lookup_table[0] = 0; distance_lookup_table[1] = 0; distance_lookup_table[2] = 1;
  }
  else if(left_distance <= DIST_THRESHHOLD && center_distance <= DIST_THRESHHOLD && right_distance > DIST_THRESHHOLD){
      digitalWrite(53, HIGH);  digitalWrite(49, HIGH);  digitalWrite(51, LOW);
      distance_lookup_table[0] = 1; distance_lookup_table[1] = 1; distance_lookup_table[2] = 0;
  }
  else if(left_distance > DIST_THRESHHOLD && center_distance <= DIST_THRESHHOLD && right_distance <= DIST_THRESHHOLD){
      digitalWrite(53, LOW);  digitalWrite(49, HIGH);  digitalWrite(51, HIGH);
      distance_lookup_table[0] = 0; distance_lookup_table[1] = 1; distance_lookup_table[2] = 1;
  }
  else if(left_distance <= DIST_THRESHHOLD && center_distance > DIST_THRESHHOLD && right_distance <= DIST_THRESHHOLD){
      digitalWrite(53, HIGH);  digitalWrite(49, LOW);  digitalWrite(51, HIGH);
      distance_lookup_table[0] = 1; distance_lookup_table[1] = 0; distance_lookup_table[2] = 1;
  }
  else if(left_distance <= DIST_THRESHHOLD && center_distance <= DIST_THRESHHOLD && right_distance <= DIST_THRESHHOLD){
      digitalWrite(53, HIGH);  digitalWrite(49, HIGH);  digitalWrite(51, HIGH);
      distance_lookup_table[0] = 1; distance_lookup_table[1] = 1; distance_lookup_table[2] = 1;
  }
}

void gombot_control(){
    
    /* ****************************************************************************
     * OBSTACLE AVOIDANCE - DOES NOT WORK CORRECTLY - PID loop is not fast enough *
     ****************************************************************************** */
    
   if(distance_lookup_table[0] == 0 && distance_lookup_table[1] == 0 && distance_lookup_table[2] == 0){
        // Reset the robot, and have it move forward
        left_motor_control  = -1;
        right_motor_control = -1;
    }
    else if(distance_lookup_table[0] == 1 && distance_lookup_table[1] == 0 && distance_lookup_table[2] == 0){
        // Obstacle to the left, begin moving to the right
        left_motor_control   = -1;
        right_motor_control -= 0.05;
    }
    else if(distance_lookup_table[0] == 0 && distance_lookup_table[1] == 1 && distance_lookup_table[2] == 0){
        // Obstacle in the center, move the robot to either the left or the right
        int time_token = millis()%2;
        if(time_token == 0){
            left_motor_control   = -1;      // Move to the left
            right_motor_control -= 0.05;
        }
        else{
            left_motor_control -= 0.05;      // Move to the right
            right_motor_control = -1;
        }
    }
    else if(distance_lookup_table[0] == 0 && distance_lookup_table[1] == 0 && distance_lookup_table[2] == 1){
        // Obstacle to the right
        left_motor_control -= 0.05;      // Move to the right
        right_motor_control = -1;
    }
    else if(distance_lookup_table[0] == 1 && distance_lookup_table[1] == 1 && distance_lookup_table[2] == 0){
        // Obstacle to left and center
        left_motor_control  = -1;      // Move to the right more aggressively
        right_motor_control -= 0.075;
    }
    else if(distance_lookup_table[0] == 1 && distance_lookup_table[1] == 0 && distance_lookup_table[2] == 1){
        // Obstacle to left and right (Hope this doesn't happen)
        left_motor_control  = -1;      // Move to the right more aggressively
        right_motor_control -= 0.075;
    }
    else if(distance_lookup_table[0] == 1 && distance_lookup_table[1] == 1 && distance_lookup_table[2] == 1){
        // Obstacles all around
        left_motor_control  -= 0.075;      // Turn around
        right_motor_control -= 0.075;
    }
    else{
        left_motor_control  = 0;      	   // Turn around
        right_motor_control = 0;      
    }
}

void SRF_poll(){
  int SRF_time_diff = (millis() - SRF_previous_time);
  if(SRF_time_diff >= 50){
    if(SRF_poll_check == 0){
      left_distance = left_distance_sensor.getDistanceCentimeter(); 
      SRF_poll_check++;
    }
    else if(SRF_poll_check == 1){
      center_distance = center_distance_sensor.getDistanceCentimeter();
      SRF_poll_check++;
    }
    else if(SRF_poll_check == 2){
      right_distance = right_distance_sensor.getDistanceCentimeter(); 
      SRF_poll_check++;
    }
    else{
      // Carry out PID constant recalculation 
      SRF_poll_check = 0;
      check_obstacles(left_distance, center_distance, right_distance);
      gombot_control();
    }
    SRF_previous_time = millis();
  }
}

// -- IMU Interrupt --//
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    /* Ultrasonic Distance Sensors
    **** USED IN OBS. AVOIDANCE SETUP, DOES NOT FUNCTION!!! *****	
    left_distance_sensor.begin(42,44);
    right_distance_sensor.begin(48,46);
    center_distance_sensor.begin(50,52); */
  
    //digitalWrite(51, HIGH);
  
    // PID Initialization
    pid_setpoint = 0;
    gombot_PID.SetMode(AUTOMATIC); 
    gombot_PID.SetOutputLimits(-PID_MAX, PID_MAX);
    gombot_PID.SetSampleTime(PID_SAMPLE_TIME);
    md.init();
    
    // Join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // Initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    /* Wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // Load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    //delay(1500);
    devStatus = mpu.dmpInitialize();

    // Sets IMU calibration values for accelerometer and gyroscope values
    // Calibration values obtained from a separate sketch. Code will be provided.
    mpu.setXGyroOffset(-247);
    mpu.setYGyroOffset(-10);
    mpu.setZGyroOffset(56);
    
    mpu.setXAccelOffset(1404); 
    mpu.setYAccelOffset(2913); 
    mpu.setZAccelOffset(1542); 

    // Make sure it worked (returns 0 if so)
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(5, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
  /* **********************************************************
     *** COMMENTED OUT CODE CORRESPONDS TO DEBUG STATEMENTS *** 
     ********************************************************** */
	
  int start_time = millis();
    // If programming failed, don't try to do anything
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            // IMPORTANT: CANNOT SET OFFSETS BECAUSE YPR VALUES ARE NOT DEGREES
            // CHANGE OFFSETS TO RADIANS OR SEE HOW THE ROBOT REACTS TO DEGREE ANGLES
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif
    }
    
    /* ***
    Serial.print("\nLeft Distance in centimers: ");    Serial.print(left_distance);
    Serial.print("\nCenter Distance in centimers: ");  Serial.print(center_distance);
    Serial.print("\nRight Distance in centimers: ");   Serial.print(right_distance);
    Serial.print("\n");
    Serial.print("Runtime: "); Serial.print(end_time - start_time); Serial.print("\n"); 
    *** */
    
    //SRF_poll();
    pid_input = ypr[2];
    gombot_PID.Compute();
    
    //Serial.print("Left Motor Value: "); Serial.print(left_motor_control); Serial.print("\n");
    //Serial.print("Right Motor Value: "); Serial.print(right_motor_control); Serial.print("\n");
    
    //delay(100);
    //int end_time = millis();
    //Serial.print(end_time - start_time); Serial.print("\n");
    md.setSpeeds(right_motor_control*pid_output, left_motor_control*pid_output);
}
