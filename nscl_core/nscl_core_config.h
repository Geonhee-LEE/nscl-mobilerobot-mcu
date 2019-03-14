/*******************************************************************************
* Copyright 2018 SEOULTECH CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Geonhee Lee */

#ifndef NSCL_CORE_CONFIG_H_
#define NSCL_CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <zetabank_msgs/SensorState.h>
#include <zetabank_msgs/Sound.h>
#include <zetabank_msgs/VersionInfo.h>

#include <Zetabank.h>

#include <math.h>


#define CONTROL_MOTOR_SPEED_PERIOD          30   //hz
#define IMU_PUBLISH_PERIOD                  200  //hz
#define CMD_VEL_PUBLISH_PERIOD              30   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD    30   //hz
#define VERSION_INFORMATION_PUBLISH_PERIOD  1    //hz 

#define WHEEL_NUM                        2
#define WHEEL_RADIUS                     0.0812           // meter
#define WHEEL_SEPARATION                 0.360           // meter (BURGER : 0.160, WAFFLE : 0.287, zetabank : 0.360)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define MAX_LINEAR_VELOCITY              2.0   // m/s   (BURGER : 0.22, WAFFLE : 0.25)
#define MAX_ANGULAR_VELOCITY             2.0   // rad/s (BURGER : 2.84, WAFFLE : 1.82)

#define TICK2RAD                         0.000255603  // (0.02929[deg] * 3.14159265359 / 180) /2(pully_) = 0.000511207f(rad)

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define VELOCITY_UNIT                    2

#define LINEAR_X_MAX_VELOCITY            1.5

#define PI 3.141592f
#define FORWARD_DIR true
#define BACKWARD_DIR false
#define STOP 2

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))  

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishCmdVelFromRC100Msg(void);
void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros);

void updateVariable(void);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(void);
void updateGoalVelocity(void);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Zetabank
zetabank_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Zetabank
zetabank_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("version_info", &version_info_msg);

// IMU of Zetabank
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Zetabank
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Zetabank
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Zetabank
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Ultrasonic sensor of Zetabank
sensor_msgs::Range sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg);



/*******************************************************************************
* For debugging
*******************************************************************************/ 
std_msgs::Float64 left_vel_msg;
ros::Publisher left_vel_pub("left_vel", &left_vel_msg);
std_msgs::Float64 right_vel_msg;
ros::Publisher right_vel_pub("right_vel", &right_vel_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Zetabank
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Zetabank
*******************************************************************************/
static uint32_t tTime[5];


/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t g_last_diff_tick[WHEEL_NUM] = {0.0, 0.0};
double  g_last_rad[WHEEL_NUM]       = {0.0, 0.0};

void updateMotorInfo(int32_t left_tick, int32_t right_tick);


/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
ZetabankSensor sensors;


/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity[VELOCITY_UNIT] = {0.0, 0.0};
float goal_velocity_from_cmd[VELOCITY_UNIT] = {0.0, 0.0};


/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
ZetabankDiagnosis diagnosis;


/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];


/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;



/*******************************************************************************
* Declaration for Zetabank
*******************************************************************************/
/*******************************************************************************
* Encoder of Zetabank 
*******************************************************************************/
int8_t enc_cnt;
long unsigned int n_left_enc_A_pulse;
long unsigned int n_left_enc_B_pulse;
long unsigned int n_left_enc_A_pulse_mean;
long unsigned int n_left_enc_B_pulse_mean;
long unsigned int n_right_enc_A_pulse;
long unsigned int n_right_enc_B_pulse;
long unsigned int n_right_enc_A_pulse_mean;
long unsigned int n_right_enc_B_pulse_mean;

unsigned int n_left_odom_A_cnt;
unsigned int n_left_odom_B_cnt;
unsigned int n_right_odom_A_cnt;
unsigned int n_right_odom_B_cnt;

void left_EncoderRead_A();
void left_EncoderRead_B();
void right_EncoderRead_A();
void right_EncoderRead_B();

void timerInterrupt(); 

bool writeVelocity(float left_vel, float right_vel);


/*******************************************************************************
* Ultrasonic of Zetabank 
*******************************************************************************/
bool ultra_flg;

void ultrasonic_check();
void ultrasonic_setup();



/*******************************************************************************
* PWM of Zetabank 
*******************************************************************************/
bool left_dir_flg;
bool right_dir_flg;

float duty_ratio;
float Kp;
float P_controll;
float pid_value, P_control, left_pid_val, right_pid_val;

float left_wheel_linear_vel;
float left_wheel_vel_err;

float right_wheel_linear_vel;
float right_wheel_vel_err;

void pwm_setup();
float pid_control(uint8_t side, float ref_vel);
void motor_pwm(uint8_t side, int8_t duty_ratio);
bool controlMotor(const float wheel_separation, float* value);
/*******************************************************************************

*******************************************************************************/



/*******************************************************************************
* For safety
*******************************************************************************/ 

void network_disconnect();
void resetGoalVelocity();


/*******************************************************************************

*******************************************************************************/

#endif // ZETABANK_CORE_CONFIG_H_
