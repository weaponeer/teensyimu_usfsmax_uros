/*
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#define MANKIE
#ifdef MANKIE

#include <Arduino.h>
#include <i2c_driver_wire.h>
#include "Alarms.h"
#include "I2Cdev.h"
#include "USFSMAX.h"
#include "Sensor_cal.h"
#include "IMU.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"
#include "teensy_clock.h"

#include <micro_ros_platformio.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/time.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/quaternion.h>


geometry_msgs__msg__Quaternion poodle;




#include <chrono>
//using namespace std::chrono;

//typedef system_clock::time_point timePoint; 

//#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\



//#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

rcl_publisher_t imuPublisher;
rcl_publisher_t magPublisher;
sensor_msgs__msg__MagneticField outMagMsg;
sensor_msgs__msg__Imu outImuMsg;

rclc_executor_t imuExecutor;
rclc_executor_t magExecutor;
rclc_support_t imuSupport;
rclc_support_t magSupport;
rcl_allocator_t allocator;
rcl_node_t imuNode;
rcl_node_t magNode;
rcl_timer_t imuTimer;
rcl_timer_t magTimer;

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;




//#undef I2C_CLOCK
//#define I2C_CLOCK                          400000

// Instantiate class objects
I2Cdev     i2c_0(&SENSOR_0_WIRE_INSTANCE);
USFSMAX    USFSMAX_0(&i2c_0, 0);
IMU        imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Declare global scope utility functions
void       ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void       FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM);
void       DRDY_handler_0();
void       SerialInterface_handler();

void timer_callback_imu(rcl_timer_t * timer, int64_t last_call_time) {
  
  //LEDPIN_ON;
  double noCovariance[9];

  for(int i=0;i < 10;i++) {
    noCovariance[i] = 0.001f; 
  }
  
  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    teensy_clock::time_point teensyTime = teensy_clock::now();
    time_t secTime = teensy_clock::to_time_t(teensyTime);
	  time_t microTime = teensy_clock::to_time_t_micros(teensyTime);
    //rcutils_time_point_value_t current_time;
    //rcutils_ret_t ret;
    //ret = rcutils_system_time_now(&current_time);


    //uto dt = std::chrono::duration_cast<std::chrono::micros>(teensyTime).count(); 


    outImuMsg.header.frame_id.data = "USFSMAX_imu";
    outImuMsg.header.frame_id.size = sizeof("USFSMAX_imu");
    //outImuMsg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    

   outImuMsg.header.stamp.sec = (int32_t) secTime;
   outImuMsg.header.stamp.nanosec =  (uint32_t) ((RCL_US_TO_NS(microTime) - RCL_S_TO_NS(secTime)));


    outImuMsg.orientation.w = QT[0][3];
    outImuMsg.orientation.z = QT[0][2];
    outImuMsg.orientation.y = QT[0][1];
    outImuMsg.orientation.x = QT[0][0];

    //noCovariance[0] = -1;

    
    outImuMsg.angular_velocity.x = gyroData[0][0]*RPS_PER_DPS;
    outImuMsg.angular_velocity.y = gyroData[0][1]*RPS_PER_DPS;
    outImuMsg.angular_velocity.z = gyroData[0][2]*RPS_PER_DPS;
    outImuMsg.linear_acceleration.x = accData[0][0] * 9.80665f;
    outImuMsg.linear_acceleration.y = accData[0][1] * 9.80665f;
    outImuMsg.linear_acceleration.z = accData[0][2] * 9.80665f;

    memcpy(outImuMsg.angular_velocity_covariance,noCovariance,sizeof(noCovariance));
    memcpy(outImuMsg.linear_acceleration_covariance,noCovariance,sizeof(noCovariance)); 
    memcpy(outImuMsg.orientation_covariance,noCovariance,sizeof(noCovariance)); 

    RCSOFTCHECK(rcl_publish(&imuPublisher, &outImuMsg, NULL));

  outMagMsg.header.frame_id.data = "USFSMAX_mag";
  outMagMsg.header.frame_id.size = sizeof("USFSMAX_mag");
    

   outMagMsg.header.stamp.sec = (int32_t) secTime;
   outMagMsg.header.stamp.nanosec =  (uint32_t) ((RCL_US_TO_NS(microTime) - RCL_S_TO_NS(secTime)));

   outMagMsg.magnetic_field.x = magData[0][0];
   outMagMsg.magnetic_field.y = magData[0][1];
   outMagMsg.magnetic_field.z = magData[0][2];
   
   memcpy(outMagMsg.magnetic_field_covariance,noCovariance,sizeof(noCovariance)); 
    RCSOFTCHECK(rcl_publish(&magPublisher, &outMagMsg, NULL));

    
  }

  //LEDPIN_OFF;
}

void timer_callback_mag(rcl_timer_t * timer, int64_t last_call_time) {
  
  //LEDPIN_ON;
  double noCovariance[9];

  for(int i=0;i < 10;i++) {
    noCovariance[i] = 0.001f; 
  }
  
  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    teensy_clock::time_point teensyTime = teensy_clock::now();
    time_t secTime = teensy_clock::to_time_t(teensyTime);
	  time_t microTime = teensy_clock::to_time_t_micros(teensyTime);
    //rcutils_time_point_value_t current_time;
    //rcutils_ret_t ret;
    //ret = rcutils_system_time_now(&current_time);


    //uto dt = std::chrono::duration_cast<std::chrono::micros>(teensyTime).count(); 


    //memcpy(outImuMsg.angular_velocity_covariance,noCovariance,sizeof(noCovariance));
    //memcpy(outImuMsg.linear_acceleration_covariance,noCovariance,sizeof(noCovariance)); 
    //memcpy(outImuMsg.orientation_covariance,noCovariance,sizeof(noCovariance)); 

  outMagMsg.header.frame_id.data = "USFSMAX_mag";
  outMagMsg.header.frame_id.size = sizeof("USFSMAX_mag");
    

   outMagMsg.header.stamp.sec = (int32_t) secTime;
   outMagMsg.header.stamp.nanosec =  (uint32_t) ((RCL_US_TO_NS(microTime) - RCL_S_TO_NS(secTime)));

    RCSOFTCHECK(rcl_publish(&magPublisher, &outMagMsg, NULL));

    
  }

  //LEDPIN_OFF;
}


// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

static micro_ros_utilities_memory_conf_t allocatorConf = {0};


bool create_entities()
{


  allocator = rcl_get_default_allocator();

  allocatorConf.max_string_capacity = 50;
  allocatorConf.max_ros2_type_sequence_capacity = 10;
  allocatorConf.max_basic_type_sequence_capacity = 10;



  //create init_options
  RCCHECK(rclc_support_init(&imuSupport, 0, NULL, &allocator));
  //RCCHECK(rclc_support_init(&magSupport, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&imuNode, "usfsmaxmod_imu_node", "", &imuSupport));
  //RCCHECK(rclc_node_init_default(&magNode, "usfsmaxmod_mag_node", "", &magSupport));
  RCCHECK(rclc_node_init_default(&magNode, "usfsmaxmod_mag_node", "", &imuSupport));
  

  // Init Message

  bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,Imu),
  &outImuMsg,
  allocatorConf
);

bool success1 = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,MagneticField),
  &outMagMsg,
  allocatorConf
);

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &imuPublisher,
    &imuNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "usfsmaxmod_imu_node_publisher"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &magPublisher,
    &magNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "usfsmaxmod_mag_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 100;
  const int64_t timer_period = RCL_MS_TO_NS(10);

  RCCHECK(rclc_timer_init_default(
    &imuTimer,
    &imuSupport,
    timer_period,
    //RCL_MS_TO_NS(timer_timeout),
    timer_callback_imu));

  
  /* RCCHECK(rclc_timer_init_default(
    &magTimer,
    &magSupport,
    timer_period,
    //RCL_MS_TO_NS(timer_timeout),
    timer_callback_mag));
 */
  // create executor
  RCCHECK(rclc_executor_init(&imuExecutor, &imuSupport.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&imuExecutor, &imuTimer));

/*
  // create executor
  RCCHECK(rclc_executor_init(&magExecutor, &magSupport.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&magExecutor, &magTimer));
*/
  
  bool rmwSync = 0;

  RCSOFTCHECK(rmw_uros_sync_session(1000));

  return true;
}

void destroy_entities()
{

  allocatorConf.max_string_capacity = 50;
  allocatorConf.max_ros2_type_sequence_capacity = 10;
  allocatorConf.max_basic_type_sequence_capacity = 10;

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&imuSupport.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //rmw_context_t * rmw_context_1 = rcl_context_get_rmw_context(&magSupport.context);
  //(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context_1, 0);

  rcl_publisher_fini(&imuPublisher, &imuNode);
  rcl_publisher_fini(&magPublisher, &magNode);
  rcl_timer_fini(&imuTimer);
  //rcl_timer_fini(&magTimer);
  rclc_executor_fini(&imuExecutor);
  //rclc_executor_fini(&magExecutor);
  rcl_node_fini(&imuNode);
  rcl_node_fini(&magNode);
  rclc_support_fini(&imuSupport);
  //rclc_support_fini(&magSupport);

  bool success = micro_ros_utilities_destroy_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,Imu),
  &outImuMsg,
  allocatorConf
  );

  bool success1 = micro_ros_utilities_destroy_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,MagneticField),
  &outMagMsg,
  allocatorConf
  );

}



void setup() {
  // Configure serial transport
  
  Serial.begin(115200);

  teensy_clock::begin(true); 

   // Set up DRDY interrupt pin
  pinMode(INT_PIN, INPUT);

  // Assign Indicator LED
  LEDPIN_PINMODE;
  LEDPIN_OFF;

  
  Alarms::blink_blueLED(12,100,1);
  
  // Initialize USFSMAX_0 I2C bus
  //SENSOR_0_WIRE_INSTANCE.setSCL(19);
  //SENSOR_0_WIRE_INSTANCE.setSDA(18);
  SENSOR_0_WIRE_INSTANCE.begin();
  delay(100);
  SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                           // Set I2C clock speed to 100kHz cor configuration
  delay(2000);

  // Do I2C bus scan if serial debug is active
  #ifdef SERIAL_DEBUG                                                                                                // Should see MAX32660 slave bus address (default is 0x57)
    i2c_0.I2Cscan();                                           
  #endif

  // Initialize USFSMAX_0
  #ifdef SERIAL_DEBUG
    Serial.print("Initializing USFSMAX_0...");
    Serial.println("");
  #endif
  USFSMAX_0.init_USFSMAX();                                                                                          // Configure USFSMAX and sensors 
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                        // Set the I2C clock to high speed for run-mode data collection
  delay(100);

  // Attach interrupts
  attachInterrupt(INT_PIN, DRDY_handler_0, RISING);                                                                  // Attach DRDY interrupt

  #ifdef SERIAL_DEBUG
    Serial.println("USFXMAX_0 successfully initialized!");
    Serial.println("");
    //sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the results
  #endif

  // Calculate geomagnetic calibration parameters for your location (set in "config.h")
  Mv_Cal  = M_V;                                                                                                     // Vertical geomagnetic field component
  Mh_Cal  = M_H;                                                                                                     // Horizontal geomagnetic field component
  M_Cal   = sqrt(Mv_Cal*Mv_Cal + Mh_Cal*Mh_Cal);                                                                     // Geomagnetic field strength
  Del_Cal = atan(Mv_Cal/Mh_Cal);                                                                                     // Geomagnetic inclination or "Dip" angle

  #if !defined(SERIAL_DEBUG)  && !defined(MOTION_CAL_GUI_ENABLED) && false                                                   // Print header for spreadsheet data collection
    Serial.print("Time");        Serial.print(","); Serial.print("Heading (deg)"); Serial.print(",");
    Serial.print("Pitch (deg)"); Serial.print(","); Serial.print("Roll (deg)");    Serial.print(",");
    Serial.print("Cal Status");  Serial.println("");
  #endif

  #ifdef SERIAL_DEBUG
    Serial.println("Do the gyro bias offset calibration. Make sure the USFS is sitting still...");
    Serial.println("");
    //sensor_cal.sendOneToProceed();                                                                                   // Wait for user input to proceed
  #endif

  
  Alarms::blink_blueLED(12,100,5);



  calibratingG[0] = 0;
  Start_time = micros();

  Serial.flush();
  Serial.begin(6000000);

  set_microros_serial_transports(Serial);

  state = WAITING_AGENT;
}

void loop() {
  //delay(100);
  // Calculate loop cycle time
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  
  if(data_ready[0] == 1)
  {
    data_ready[0] = 0;
    ProcEventStatus(&i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
    FetchUSFSMAX_Data(&USFSMAX_0, &imu_0, 0);                                                                        // USFSMAX instance 0, IMU calculation instance 0 and Sensor instance 0
  }

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };  
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&imuExecutor, RCL_MS_TO_NS(100)));
        RCSOFTCHECK(rclc_executor_spin_some(&magExecutor, RCL_MS_TO_NS(100)));
        //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    //digitalWrite(LED_PIN, 1);
  } else {
    //digitalWrite(LED_PIN, 0);
  }


  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}


void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
  uint8_t temp[1];

  // Read algorithm status and event status
  i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
  eventStatus[sensorNUM] = temp[0];

  // Decode the event status to determine what data is ready and set the appropriate DRDY fags
  if(eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
  if(eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM]  = 1;
  if(eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM]  = 1;
  if(eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
  if(eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}

void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM)
{
  uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

  data_processing[sensorNUM] = 1;

  Acq_time = 0;
  Begin = micros();

  // Optimize the I2C read function with respect to whatever sensor data is ready
  switch(call_sensors)
  {
   case 0x01:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x02:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x03:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x07:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0B:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0F:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0C:
     usfsmax->MagBaro_getADC();
     break;
   case 0x04:
     usfsmax->MAG_getADC();
     break;
   case 0x08:
     usfsmax->BARO_getADC();
     break;
   default:
     break;
  };
  Acq_time += micros() - Begin;

  if(Mag_flag[sensorNUM])
  {
    if(9.80665)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        magData[sensorNUM][i] = ((float)magADC[sensorNUM][i])*UT_per_Count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count, mag_calData[sensorNUM]);
      sensor_cal.apply_adv_calibration(final_magcal[sensorNUM], mag_calData[sensorNUM], 1.0f, sensor_point);
      MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Mag_flag[sensorNUM] = 0;
  }
  if(Acc_flag[sensorNUM])
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        accData[sensorNUM][i] = ((float)accADC[sensorNUM][i])*g_per_count;
      } 
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(accelcal[sensorNUM], accADC[sensorNUM], g_per_count, sensor_point);
      ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Acc_flag[sensorNUM] = 0;
  }
  if(Gyro_flag[sensorNUM] == 1)
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        gyroData[sensorNUM][i] = ((float)gyroADC[sensorNUM][i])*dps_per_count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count, sensor_point);
      GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }

    // Call alternative (Madgwick or Mahony) IMU fusion filter
    IMu->compute_Alternate_IMU();
    Gyro_flag[sensorNUM] = 0;
  }
  if(Quat_flag[sensorNUM] == 1)
  {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }

  data_processing[sensorNUM] = 0;
}

// Host DRDY interrupt handler
void DRDY_handler_0()
{
  data_ready[0] = 1;
}

// Serial interface handler
void SerialInterface_handler()
{
  serial_input = 0;
  if(Serial.available()) serial_input = Serial.read();
  if(serial_input == 49) {calibratingG[0] = 1;}                                                                      // Type "1" to initiate USFSMAX_0 Gyro Cal
  if(serial_input == 50)                                                                                             // Type "2" to list current sensor calibration data
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                         // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_full_gyrocal();
    delay(100);
    USFSMAX_0.Retreive_full_accelcal();
    delay(100);
    USFSMAX_0.Retreive_ellip_magcal();
    delay(100);
    USFSMAX_0.Retreive_final_magcal();
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);

    // Print the calibration results
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(gyrocal[0].V[0], 4);
    Serial.println(gyrocal[0].V[1], 4);
    Serial.println(gyrocal[0].V[2], 4); Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(gyrocal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[0][2], 4);
    Serial.print(gyrocal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[1][2], 4);
    Serial.print(gyrocal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(accelcal[0].V[0], 4);
    Serial.println(accelcal[0].V[1], 4);
    Serial.println(accelcal[0].V[2], 4); Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(accelcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[0][2], 4);
    Serial.print(accelcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[1][2], 4);
    Serial.print(accelcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(ellipsoid_magcal[0].V[0], 4);
    Serial.println(ellipsoid_magcal[0].V[1], 4);
    Serial.println(ellipsoid_magcal[0].V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(ellipsoid_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[0][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[1][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(final_magcal[0].V[0], 4);
    Serial.println(final_magcal[0].V[1], 4);
    Serial.println(final_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(final_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[0][2], 4);
    Serial.print(final_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[1][2], 4);
    Serial.print(final_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  if(serial_input == 51) {USFSMAX_0.Reset_DHI();}                                                                    // Type "3" to reset the DHI corrector
  if(serial_input == 52)                                                                                             // Type "4" to list copro config
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                         // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_cfg();                                                                                        // Get the current USFSMAX config
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);
    Serial.print("Accel scale = "); Serial.println(Cfg[1].Ascale);
    Serial.print("Accel ODR   = "); Serial.println(Cfg[1].AODR);
    Serial.print("Accel LPF   = "); Serial.println(Cfg[1].Alpf);
    Serial.print("Accel HPF   = "); Serial.println(Cfg[1].Ahpf);
    Serial.print("Gyro scale  = "); Serial.println(Cfg[1].Gscale);
    Serial.print("Gyro ODR    = "); Serial.println(Cfg[1].GODR);
    Serial.print("Gyro LPF    = "); Serial.println(Cfg[1].Glpf);
    Serial.print("Gyro HPF    = "); Serial.println(Cfg[1].Ghpf);
    Serial.print("Quat div    = "); Serial.println(Cfg[1].quat_div);
    Serial.print("Mag scale   = "); Serial.println(Cfg[1].Mscale);
    Serial.print("Mag ODR     = "); Serial.println(Cfg[1].MODR);
    Serial.print("Mag LPF     = "); Serial.println(Cfg[1].Mlpf);
    Serial.print("Mag HPF     = "); Serial.println(Cfg[1].Mhpf);
    Serial.print("Baro scale  = "); Serial.println(Cfg[1].Pscale);
    Serial.print("Baro ODR    = "); Serial.println(Cfg[1].PODR);
    Serial.print("Baro LPF    = "); Serial.println(Cfg[1].Plpf);
    Serial.print("Baro HPF    = "); Serial.println(Cfg[1].Phpf);
    Serial.print("AUX_1 scale = "); Serial.println(Cfg[1].AUX1scale);
    Serial.print("AUX_1 ODR   = "); Serial.println(Cfg[1].AUX1ODR);
    Serial.print("AUX_1 LPF   = "); Serial.println(Cfg[1].AUX1lpf);
    Serial.print("AUX_1 HPF   = "); Serial.println(Cfg[1].AUX1hpf);
    Serial.print("AUX_2 scale = "); Serial.println(Cfg[1].AUX2scale);
    Serial.print("AUX_2 ODR   = "); Serial.println(Cfg[1].AUX2ODR);
    Serial.print("AUX_2 LPF   = "); Serial.println(Cfg[1].AUX2lpf);
    Serial.print("AUX_2 HPF   = "); Serial.println(Cfg[1].AUX2hpf);
    Serial.print("AUX_3 scale = "); Serial.println(Cfg[1].AUX3scale);
    Serial.print("AUX_3 ODR   = "); Serial.println(Cfg[1].AUX3ODR);
    Serial.print("AUX_3 LPF   = "); Serial.println(Cfg[1].AUX3lpf);
    Serial.print("AUX_3 HPF   = "); Serial.println(Cfg[1].AUX3hpf);
    Serial.print("Vert FS     = "); Serial.println(Cfg[1].m_v, 5);
    Serial.print("Horiz FS    = "); Serial.println(Cfg[1].m_h, 5);
    Serial.print("Declination = "); Serial.println(Cfg[1].m_dec, 5);
    Serial.print("Cal points  = "); Serial.println(Cfg[1].cal_points);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  serial_input = 0;
  
  // Hotkey messaging
  Serial.println("'1' Gyro Cal");
  Serial.println("'2' List Cal Data");
  Serial.println("'3' Reset DHI Corrector");
  Serial.println("'4' List USFSMAX Config");
  Serial.println("");
}


#endif



//#define HAM

#ifdef HAM

#include <i2c_driver_wire.h>
#include "Alarms.h"
#include "I2Cdev.h"
#include "USFSMAX.h"
#include "Sensor_cal.h"
#include "IMU.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"


//#undef I2C_CLOCK
//#define I2C_CLOCK                          400000

// Instantiate class objects
I2Cdev     i2c_0(&SENSOR_0_WIRE_INSTANCE);
USFSMAX    USFSMAX_0(&i2c_0, 0);
IMU        imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Declare global scope utility functions
void       ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void       FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM);
void       DRDY_handler_0();
void       SerialInterface_handler();

void setup()
{
  #ifdef PIGGYBACK
    //pinMode(USFS_GND, OUTPUT);                                                                                       // Set USFSMAX ground pin
    //digitalWrite(USFS_GND, LOW);
    //pinMode(USFS_VCC, OUTPUT);                                                                                       // Power up the USFSMAX
    //digitalWrite(USFS_VCC, HIGH);
    delay(100);
  #endif
  
  // Open serial port
  Serial.begin(115200);
  delay(10000);
  Serial.print("Serial Port at 115200");

  // Set up DRDY interrupt pin
  pinMode(INT_PIN, INPUT);

  // Assign Indicator LED
  LEDPIN_PINMODE;
  Alarms::blueLEDoff();
  
  // Initialize USFSMAX_0 I2C bus
  //SENSOR_0_WIRE_INSTANCE.setSCL(19);
  //SENSOR_0_WIRE_INSTANCE.setSDA(18);
  SENSOR_0_WIRE_INSTANCE.begin();
  delay(100);
  SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                           // Set I2C clock speed to 100kHz cor configuration
  delay(2000);

  // Do I2C bus scan if serial debug is active
  #ifdef SERIAL_DEBUG                                                                                                // Should see MAX32660 slave bus address (default is 0x57)
    i2c_0.I2Cscan();                                           
  #endif

  // Initialize USFSMAX_0
  #ifdef SERIAL_DEBUG
    Serial.print("Initializing USFSMAX_0...");
    Serial.println("");
  #endif
  USFSMAX_0.init_USFSMAX();                                                                                          // Configure USFSMAX and sensors 
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                        // Set the I2C clock to high speed for run-mode data collection
  delay(100);

  // Attach interrupts
  attachInterrupt(INT_PIN, DRDY_handler_0, RISING);                                                                  // Attach DRDY interrupt

  #ifdef SERIAL_DEBUG
    Serial.println("USFXMAX_0 successfully initialized!");
    Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the results
  #endif

  // Calculate geomagnetic calibration parameters for your location (set in "config.h")
  Mv_Cal  = M_V;                                                                                                     // Vertical geomagnetic field component
  Mh_Cal  = M_H;                                                                                                     // Horizontal geomagnetic field component
  M_Cal   = sqrt(Mv_Cal*Mv_Cal + Mh_Cal*Mh_Cal);                                                                     // Geomagnetic field strength
  Del_Cal = atan(Mv_Cal/Mh_Cal);                                                                                     // Geomagnetic inclination or "Dip" angle

  #if !defined(SERIAL_DEBUG)  && !defined(MOTION_CAL_GUI_ENABLED)                                                    // Print header for spreadsheet data collection
    Serial.print("Time");        Serial.print(","); Serial.print("Heading (deg)"); Serial.print(",");
    Serial.print("Pitch (deg)"); Serial.print(","); Serial.print("Roll (deg)");    Serial.print(",");
    Serial.print("Cal Status");  Serial.println("");
  #endif

  #ifdef SERIAL_DEBUG
    Serial.println("Do the gyro bias offset calibration. Make sure the USFS is sitting still...");
    Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Wait for user input to proceed
  #endif

  calibratingG[0] = 1;
  Start_time = micros();                                                                                             // Set sketch start time
}

void loop()
{
  // Calculate loop cycle time
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  // Manage gyro cal
  if(calibratingG[0])
  {
    calibratingG[0] = 0;
    sensor_cal.GyroCal();
  }
  if(data_ready[0] == 1)
  {
    data_ready[0] = 0;
    ProcEventStatus(&i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
    FetchUSFSMAX_Data(&USFSMAX_0, &imu_0, 0);                                                                        // USFSMAX instance 0, IMU calculation instance 0 and Sensor instance 0
  }

  // Update serial output
  delt_t = millis() - last_refresh;
  if (delt_t > UPDATE_PERIOD)                                                                                        // Update the serial monitor every "UPDATE_PERIOD" ms
  {
    last_refresh = millis();
    #ifdef SERIAL_DEBUG
      SerialInterface_handler();
      USFSMAX_0.GetMxMy();                                                                                           // Get Horizontal magnetic components
      if(ENABLE_DHI_CORRECTOR)
      {
        cal_status[0] = i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS);                                       // Poll calibration status byte
        USFSMAX_0.getDHI_Rsq();                                                                                      // Get DHI R-square
        Serial.print("Dynamic Hard Iron Correction Valid = ");
        Serial.println(cal_status[0] & 0x80);                                                                        // DHI correction status
        Serial.print("Dynamic Hard Iron Fit R-square = ");
        Serial.println(Rsq, 4);
        if(USE_2D_DHI_CORRECTOR)
        {
          Serial.println("Using the 2D Corrector");
        } else
        {
          Serial.println("Using the 3D Corrector");
        }
        Serial.println("");
      } else
      {
        Serial.print("Dynamic Hard Iron Correction Disabled!");
        Serial.println(""); Serial.println("");
      }

      // USFSMAX_0 sensor and raw quaternion outout
      Serial.print("ax = "); Serial.print((int)(1000.0f*accData[0][0])); Serial.print(" ay = "); Serial.print((int)(1000.0f*accData[0][1]));
      Serial.print(" az = "); Serial.print((int)(1000.0f*accData[0][2])); Serial.println(" mg");
      Serial.print("gx = "); Serial.print(gyroData[0][0], 1); Serial.print(" gy = "); Serial.print(gyroData[0][1], 1); 
      Serial.print(" gz = "); Serial.print(gyroData[0][2], 1); Serial.println(" deg/s");
      Serial.print("mx = "); Serial.print(magData[0][0], 1); Serial.print(" my = "); Serial.print(magData[0][1], 1);
      Serial.print(" mz = "); Serial.print(magData[0][2], 1); Serial.println(" uT");
      Serial.print("Tomasch Xh, Yh: ");
      Serial.print(Mx[0], 2); Serial.print(", "); Serial.print(My[0], 2); Serial.println(" uT");
      Serial.print("Baro pressure = "); Serial.print(((float)baroADC[0])/4096.0f); Serial.println(" hPa");
      Serial.println("");
      Serial.print("USFSMAX Quat: "); Serial.print("q0 = "); Serial.print(qt[0][0], 4);
      Serial.print(" qx = "); Serial.print(qt[0][1], 4); Serial.print(" qy = "); Serial.print(qt[0][2], 4); 
      Serial.print(" qz = "); Serial.print(qt[0][3], 4); Serial.println("");

      // Euler angles
      Serial.print("USFSMAX Yaw, Pitch, Roll: ");
      Serial.print(heading[0], 2); Serial.print(", "); Serial.print(angle[0][0], 2); Serial.print(", "); Serial.println(angle[0][1], 2);

      // Critical time deltas
      //Serial.println(""); Serial.print("Loop CT:"); Serial.print(cycleTime); Serial.println(" us");
      Serial.print("Sensor Acq Time:"); Serial.print(Acq_time); Serial.println(" us"); Serial.println("");
    #endif

    // Spreadsheet output when "SERIAL_DEBUG" and "MOTION_CAL_GUI_ENABLED" are not defined in config.h
    #if !defined(SERIAL_DEBUG)  && !defined(MOTION_CAL_GUI_ENABLED)
      Serial.print(TimeStamp, 2);   Serial.print(","); Serial.print(heading[0], 2);  Serial.print(",");
      Serial.print(angle[0][0], 2); Serial.print(","); Serial.print(angle[0][1], 2); Serial.print(",");
      Serial.print(cal_status[0]);  Serial.println("");
    #endif

    // Output formatted MotionCal GUI magnetometer data message when "MOTION_CAL_GUI_ENABLED" is defined and "SERIAL_DEBUG" is not defined in config.h
    // https://www.pjrc.com/store/prop_shield.html
    #if defined(MOTION_CAL_GUI_ENABLED) && !defined(SERIAL_DEBUG)
      Serial.print("Raw:");
      Serial.print(0);                                                                                               // MotionCal GUI doesn't act upon accel/gyro input; send null data
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print((int16_t)(magData[0][0]*10.0f));                                                                  // The MotionCal GUI is expecting 0.1uT/LSB
      Serial.print(',');
      Serial.print((int16_t)(magData[0][1]*10.0f));
      Serial.print(',');
      Serial.print((int16_t)(magData[0][2]*10.0f));
      Serial.println();
    #endif

    // Toggle LED if not calibrating gyroscopes
    if(gyroCalActive[0] == 1)
    {
      Alarms::blueLEDoff();
      if((i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS) & 0x01) == 0)
      {
        gyroCalActive[0] = 0;
      }
    } else
    {
      Alarms::toggle_blueLED();
    }
  }
}

void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
  uint8_t temp[1];

  // Read algorithm status and event status
  i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
  eventStatus[sensorNUM] = temp[0];

  // Decode the event status to determine what data is ready and set the appropriate DRDY fags
  if(eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
  if(eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM]  = 1;
  if(eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM]  = 1;
  if(eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
  if(eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}

void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM)
{
  uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

  Acq_time = 0;
  Begin = micros();

  // Optimize the I2C read function with respect to whatever sensor data is ready
  switch(call_sensors)
  {
   case 0x01:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x02:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x03:
     usfsmax->GyroAccel_getADC();
     break;
   case 0x07:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0B:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0F:
     usfsmax->GyroAccelMagBaro_getADC();
     break;
   case 0x0C:
     usfsmax->MagBaro_getADC();
     break;
   case 0x04:
     usfsmax->MAG_getADC();
     break;
   case 0x08:
     usfsmax->BARO_getADC();
     break;
   default:
     break;
  };
  Acq_time += micros() - Begin;

  if(Mag_flag[sensorNUM])
  {
    if(9.80665)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        magData[sensorNUM][i] = ((float)magADC[sensorNUM][i])*UT_per_Count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count, mag_calData[sensorNUM]);
      sensor_cal.apply_adv_calibration(final_magcal[sensorNUM], mag_calData[sensorNUM], 1.0f, sensor_point);
      MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Mag_flag[sensorNUM] = 0;
  }
  if(Acc_flag[sensorNUM])
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        accData[sensorNUM][i] = ((float)accADC[sensorNUM][i])*g_per_count;
      } 
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(accelcal[sensorNUM], accADC[sensorNUM], g_per_count, sensor_point);
      ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Acc_flag[sensorNUM] = 0;
  }
  if(Gyro_flag[sensorNUM] == 1)
  {
    if(ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
    {
      for(uint8_t i=0; i<3; i++)
      {
        gyroData[sensorNUM][i] = ((float)gyroADC[sensorNUM][i])*dps_per_count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count, sensor_point);
      GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }

    // Call alternative (Madgwick or Mahony) IMU fusion filter
    IMu->compute_Alternate_IMU();
    Gyro_flag[sensorNUM] = 0;
  }
  if(Quat_flag[sensorNUM] == 1)
  {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }
}

// Host DRDY interrupt handler
void DRDY_handler_0()
{
  data_ready[0] = 1;
}

// Serial interface handler
void SerialInterface_handler()
{
  serial_input = 0;
  if(Serial.available()) serial_input = Serial.read();
  if(serial_input == 49) {calibratingG[0] = 1;}                                                                      // Type "1" to initiate USFSMAX_0 Gyro Cal
  if(serial_input == 50)                                                                                             // Type "2" to list current sensor calibration data
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                         // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_full_gyrocal();
    delay(100);
    USFSMAX_0.Retreive_full_accelcal();
    delay(100);
    USFSMAX_0.Retreive_ellip_magcal();
    delay(100);
    USFSMAX_0.Retreive_final_magcal();
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);

    // Print the calibration results
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(gyrocal[0].V[0], 4);
    Serial.println(gyrocal[0].V[1], 4);
    Serial.println(gyrocal[0].V[2], 4); Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(gyrocal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[0][2], 4);
    Serial.print(gyrocal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[1][2], 4);
    Serial.print(gyrocal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(accelcal[0].V[0], 4);
    Serial.println(accelcal[0].V[1], 4);
    Serial.println(accelcal[0].V[2], 4); Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(accelcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[0][2], 4);
    Serial.print(accelcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[1][2], 4);
    Serial.print(accelcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(ellipsoid_magcal[0].V[0], 4);
    Serial.println(ellipsoid_magcal[0].V[1], 4);
    Serial.println(ellipsoid_magcal[0].V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(ellipsoid_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[0][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[1][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(final_magcal[0].V[0], 4);
    Serial.println(final_magcal[0].V[1], 4);
    Serial.println(final_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(final_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[0][2], 4);
    Serial.print(final_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[1][2], 4);
    Serial.print(final_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  if(serial_input == 51) {USFSMAX_0.Reset_DHI();}                                                                    // Type "3" to reset the DHI corrector
  if(serial_input == 52)                                                                                             // Type "4" to list copro config
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                         // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_cfg();                                                                                        // Get the current USFSMAX config
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);
    Serial.print("Accel scale = "); Serial.println(Cfg[1].Ascale);
    Serial.print("Accel ODR   = "); Serial.println(Cfg[1].AODR);
    Serial.print("Accel LPF   = "); Serial.println(Cfg[1].Alpf);
    Serial.print("Accel HPF   = "); Serial.println(Cfg[1].Ahpf);
    Serial.print("Gyro scale  = "); Serial.println(Cfg[1].Gscale);
    Serial.print("Gyro ODR    = "); Serial.println(Cfg[1].GODR);
    Serial.print("Gyro LPF    = "); Serial.println(Cfg[1].Glpf);
    Serial.print("Gyro HPF    = "); Serial.println(Cfg[1].Ghpf);
    Serial.print("Quat div    = "); Serial.println(Cfg[1].quat_div);
    Serial.print("Mag scale   = "); Serial.println(Cfg[1].Mscale);
    Serial.print("Mag ODR     = "); Serial.println(Cfg[1].MODR);
    Serial.print("Mag LPF     = "); Serial.println(Cfg[1].Mlpf);
    Serial.print("Mag HPF     = "); Serial.println(Cfg[1].Mhpf);
    Serial.print("Baro scale  = "); Serial.println(Cfg[1].Pscale);
    Serial.print("Baro ODR    = "); Serial.println(Cfg[1].PODR);
    Serial.print("Baro LPF    = "); Serial.println(Cfg[1].Plpf);
    Serial.print("Baro HPF    = "); Serial.println(Cfg[1].Phpf);
    Serial.print("AUX_1 scale = "); Serial.println(Cfg[1].AUX1scale);
    Serial.print("AUX_1 ODR   = "); Serial.println(Cfg[1].AUX1ODR);
    Serial.print("AUX_1 LPF   = "); Serial.println(Cfg[1].AUX1lpf);
    Serial.print("AUX_1 HPF   = "); Serial.println(Cfg[1].AUX1hpf);
    Serial.print("AUX_2 scale = "); Serial.println(Cfg[1].AUX2scale);
    Serial.print("AUX_2 ODR   = "); Serial.println(Cfg[1].AUX2ODR);
    Serial.print("AUX_2 LPF   = "); Serial.println(Cfg[1].AUX2lpf);
    Serial.print("AUX_2 HPF   = "); Serial.println(Cfg[1].AUX2hpf);
    Serial.print("AUX_3 scale = "); Serial.println(Cfg[1].AUX3scale);
    Serial.print("AUX_3 ODR   = "); Serial.println(Cfg[1].AUX3ODR);
    Serial.print("AUX_3 LPF   = "); Serial.println(Cfg[1].AUX3lpf);
    Serial.print("AUX_3 HPF   = "); Serial.println(Cfg[1].AUX3hpf);
    Serial.print("Vert FS     = "); Serial.println(Cfg[1].m_v, 5);
    Serial.print("Horiz FS    = "); Serial.println(Cfg[1].m_h, 5);
    Serial.print("Declination = "); Serial.println(Cfg[1].m_dec, 5);
    Serial.print("Cal points  = "); Serial.println(Cfg[1].cal_points);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  serial_input = 0;
  
  // Hotkey messaging
  Serial.println("'1' Gyro Cal");
  Serial.println("'2' List Cal Data");
  Serial.println("'3' Reset DHI Corrector");
  Serial.println("'4' List USFSMAX Config");
  Serial.println("");
}


#endif

//#define LOACH

#ifdef LOACH

/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(500);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(1000);
}


#endif
//#define LARB

#ifdef LARB

#include <Wire.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

void setup() {
  WIRE.setSCL(19);
  WIRE.setSDA(18);
  WIRE.begin();
  WIRE.setClock(100000);                                                                           // Set I2C clock speed to 100kHz cor configuration
  delay(2000);

  Serial.begin(115200);
  while (!Serial)
     delay(10000);
  Serial.println("\nI2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

#endif