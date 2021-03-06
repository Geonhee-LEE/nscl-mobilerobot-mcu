/*******************************************************************************
* Copyright 2018 SEOULTECH CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Geonhee Lee */

#include "nscl_core_config.h"

/*******************************************************************************
* Setup function
*******************************************************************************/

//setup Timer1
HardwareTimer Timer(TIMER_CH1);

void setup() 
{
    // Initialize ROS node handle, advertise and subscribe the topics
    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(reset_sub);
    nh.advertise(sensor_state_pub);
    nh.advertise(version_info_pub);
    nh.advertise(imu_pub);
    nh.advertise(odom_pub);
    nh.advertise(joint_states_pub);
    nh.advertise(battery_state_pub);  
    nh.advertise(sonar_pub); 
    // Debugging
    nh.advertise(right_vel_pub);
    nh.advertise(left_vel_pub);

    tf_broadcaster.init(nh);

    // Setting for IMU
    sensors.init();

    // Setting for SLAM and navigation (odometry, joint states, TF)
    initOdom();
    initJointStates();

    prev_update_time = millis();

    // For voltage check
    setup_end = true;

    // Setting additional functions
    encoder_setup();
    timer_setup();
    motor_setup();
    //ultrasonic_setup();
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop() 
{
    uint32_t t = millis();

    // if the connection of ROS communication with PC is disconnected, it will skip.
    if(nh.connected() )
    {
        updateTime();
        updateVariable();
        if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))  //CONTROL_MOTOR_SPEED_PERIOD와 같은 Hz 재정의 필요!
        {
            // Subscribe cmd_vel and control motors
            updateGoalVelocity();
            controlMotor(WHEEL_SEPARATION, goal_velocity);
            tTime[0] = t;
        }
        if ((t - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD)) 
        {
            // subscribe sensors embedded in OpenCR and update odometry infomation.
            publishSensorStateMsg();
            publishDriveInformation();
            tTime[1] = t;
        }
        if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD)) 
        {
            publishBatteryStateMsg();
            tTime[2] = t;
        }
        if ((t - tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD)) 
        {
            //update imu
            publishImuMsg();
            tTime[3] = t;
        }
        if ((t - tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_PERIOD)) 
        {
            tTime[4] = t;
        }
        // Update the IMU unit
        sensors.updateIMU();

        // Start Gyro Calibration after ROS connection
        updateGyroCali();
    }
    else
    {
        network_disconnect();
    }

    // Call all the callbacks waiting to be called at that point in time
    nh.spinOnce();

    // give the serial link time to process
    delay(10);
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(void) 
{
    static bool variable_flag = false;

    if (nh.connected()) 
    {
        if (variable_flag == false) 
        {
            sensors.initIMU();
            initOdom();
            variable_flag = true;
        }
    } 
    else 
    {
        variable_flag = false;
    }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime() 
{
    current_offset = micros();
    current_time   = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow() 
{
    return addMicros(current_time, micros() - current_offset);
}

/*******************************************************************************
* Time Interpolation function
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros) 
{
    uint32_t sec,
    nsec;

    sec  = _micros / 1000000 + t.sec;
    nsec = _micros % 1000000 + 1000 * (t.nsec / 1000);

    if (nsec >= 1e9) 
    {
        sec++,
        nsec--;
    }
    return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void) 
{
    static bool isEnded = false;
    char log_msg[50];

    if (nh.connected()) 
    {
        if (isEnded == false) 
        {
            sprintf(log_msg, "Start Calibration of Gyro");
            nh.loginfo(log_msg);
            sensors.calibrationGyro();
            sprintf(log_msg, "Calibration End");
            nh.loginfo(log_msg);
            isEnded = true;
        }
    } 
    else 
    {
        isEnded = false;
    }
}

void initOdom(void) 
{
    init_encoder = true;

    for (int index = 0; index < 3; index++) 
    {
        odom_pose[index] = 0.0;
        odom_vel[index]  = 0.0;
    }

    odom.pose.pose.position.x    = 0.0;
    odom.pose.pose.position.y    = 0.0;
    odom.pose.pose.position.z    = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;
    odom.twist.twist.linear.x    = 0.0;
    odom.twist.twist.angular.z   = 0.0;
}

void initJointStates(void) 
{
    static char * joint_states_name[] = 
    {
        "wheel_left_joint",
        "wheel_right_joint"
    };

    joint_states.header.frame_id      = "base_link";
    joint_states.name                 = joint_states_name;

    joint_states.name_length          = WHEEL_NUM;
    joint_states.position_length      = WHEEL_NUM;
    joint_states.velocity_length      = WHEEL_NUM;
    joint_states.effort_length        = WHEEL_NUM;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist & cmd_vel_msg) 
{
    goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    goal_velocity_from_cmd[LINEAR]  = constrain(
        goal_velocity_from_cmd[LINEAR],
        (-1) * MAX_LINEAR_VELOCITY,
        MAX_LINEAR_VELOCITY
    );
    goal_velocity_from_cmd[ANGULAR] = constrain(
        goal_velocity_from_cmd[ANGULAR],
        (-1) * MAX_ANGULAR_VELOCITY,
        MAX_ANGULAR_VELOCITY
    );
}


/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty & reset_msg) 
{
    char log_msg[50];

    sprintf(log_msg, "Start Calibration of Gyro");
    nh.loginfo(log_msg);
    sensors.calibrationGyro();
    sprintf(log_msg, "Calibration End");
    nh.loginfo(log_msg);
    initOdom();
    sprintf(log_msg, "Reset Odometry");
    nh.loginfo(log_msg);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void) 
{
    imu_msg                 = sensors.getIMU();
    imu_msg.header.stamp    = rosNow();
    imu_msg.header.frame_id = "imu_link";
    imu_pub.publish(& imu_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void) 
{

    // ************** Info of Time ************** //
    sensor_state_msg.header.stamp = rosNow();

    // ************** Info of Voltage ************** //
    //sensor_state_msg.battery      = checkVoltage();

    // ************** Info of Encoder************** //
    if (left_dir_flg == FORWARD_DIR && right_dir_flg == FORWARD_DIR) 
        updateMotorInfo((n_left_odom_A_cnt + n_left_odom_B_cnt) * 0.5,(n_right_odom_A_cnt + n_right_odom_B_cnt) * 0.5);
    else if (left_dir_flg == FORWARD_DIR && right_dir_flg == BACKWARD_DIR) 
        updateMotorInfo((n_left_odom_A_cnt + n_left_odom_B_cnt) * 0.5,-((n_right_odom_A_cnt + n_right_odom_B_cnt) * 0.5));
    else if (left_dir_flg == BACKWARD_DIR && right_dir_flg == FORWARD_DIR) 
        updateMotorInfo(-((n_left_odom_A_cnt + n_left_odom_B_cnt) * 0.5),(n_right_odom_A_cnt + n_right_odom_B_cnt) * 0.5);
    else if (left_dir_flg == BACKWARD_DIR && right_dir_flg == BACKWARD_DIR) 
        updateMotorInfo(-((n_left_odom_A_cnt + n_left_odom_B_cnt) * 0.5),-((n_right_odom_A_cnt + n_right_odom_B_cnt) * 0.5));
    
    sensor_state_msg.left_encoder  = (n_left_odom_A_cnt + n_left_odom_B_cnt) * 0.5;
    sensor_state_msg.right_encoder = (n_right_odom_A_cnt + n_right_odom_B_cnt) * 0.5;

    n_left_odom_A_cnt              = 0;
    n_left_odom_B_cnt              = 0;
    n_right_odom_A_cnt             = 0;
    n_right_odom_B_cnt             = 0;


    // ************** Info of Buttons ************** //
    sensor_state_msg.button        = sensors.checkPushButton();
    //sensor_state_msg.torque = motor_driver.getTorque();
    sensor_state_pub.publish(& sensor_state_msg);
 

}


/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void) 
{
    battery_state_msg.header.stamp    = rosNow();
    battery_state_msg.design_capacity = 14.55f; //Ah
    battery_state_msg.voltage         = analogRead(A0);
    battery_state_msg.percentage      = (float)(battery_state_msg.voltage * 0.004882813f);

    if (battery_state == 0) 
        battery_state_msg.present = false;
    else 
        battery_state_msg.present = true;
    
    battery_state_pub.publish(& battery_state_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void) 
{
    unsigned long time_now  = millis();
    unsigned long step_time = time_now - prev_update_time; // dimension = [msec]

    prev_update_time = time_now;
    ros::Time stamp_now     = rosNow();

    // calculate odometry
    calcOdometry((double)(step_time * 0.001)); // dimension = [sec]

    // odometry
    updateOdometry();
    odom.header.stamp = stamp_now;
    odom_pub.publish(& odom);

    // odometry tf
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    // joint states
    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(& joint_states);

}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void) 
{
    odom.header.frame_id       = "odom";
    odom.child_frame_id        = "base_link";

    odom.pose.pose.position.x  = odom_pose[0];
    odom.pose.pose.position.y  = odom_pose[1];
    odom.pose.pose.position.z  = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];

}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick) 
{
    // The period is 100ms
    int32_t current_tick                = 0;
    static int32_t last_tick[WHEEL_NUM] = { 0.0, 0.0};

    if (init_encoder) 
    {
        for (int index = 0; index < WHEEL_NUM; index++) 
        {
            g_last_diff_tick[index] = 0.0;
            last_tick[index]        = 0.0;
            g_last_rad[index]       = 0.0;

            last_velocity[index]    = 0.0;
        }

        last_tick[LEFT]  = left_tick;
        last_tick[RIGHT] = right_tick;

        init_encoder     = false;
        return;
    }

    g_last_diff_tick[LEFT] = left_tick; // difference of encoder pulses
    g_last_rad[LEFT]       += TICK2RAD * (double)g_last_diff_tick[LEFT]; // translate the number of conunt to radian


    g_last_diff_tick[RIGHT] = right_tick; // difference of encoder pulses
    g_last_rad[RIGHT]       += TICK2RAD * (double)g_last_diff_tick[RIGHT]; // translate the number of conunt to radian

}
/*******************************************************************************
* Update the joint states
*******************************************************************************/
void updateJointStates(void) 
{
    static float joint_states_pos[WHEEL_NUM] = 
    {
        0.0,
        0.0
    };
    static float joint_states_vel[WHEEL_NUM] = 
    {
        0.0,
        0.0
    };
    static float joint_states_eff[WHEEL_NUM] = 
    {
        0.0,
        0.0
    };

    joint_states_pos[LEFT]                   = g_last_rad[LEFT];
    joint_states_pos[RIGHT]                  = g_last_rad[RIGHT];

    joint_states_vel[LEFT]                   = last_velocity[LEFT];
    joint_states_vel[RIGHT]                  = last_velocity[RIGHT];

    joint_states.position                    = joint_states_pos;
    joint_states.velocity                    = joint_states_vel;
    joint_states.effort                      = joint_states_eff;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped & odom_tf) 
{
    odom_tf.header                  = odom.header;
    odom_tf.child_frame_id          = "base_footprint";
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time) 
{
    float * orientation;
    double wheel_l, wheel_r; // rotation value of wheel [rad]
    double delta_s,
    theta,
    delta_theta;
    static double last_theta = 0.0;
    double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;

    wheel_l   = wheel_r = 0.0;
    delta_s   = delta_theta = theta = 0.0;
    v         = w = 0.0;
    step_time = 0.0;

    step_time = diff_time; // dimension : [sec]
    double step_freq = 1/step_time;

    if (step_time == 0) 
        return false;
    
    wheel_l = TICK2RAD * (double)g_last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double)g_last_diff_tick[RIGHT];

    if (isnan(wheel_l)) 
        wheel_l = 0.0;
    if (isnan(wheel_r)) 
        wheel_r = 0.0;
    
    delta_s              = WHEEL_RADIUS * (wheel_r + wheel_l) * 0.50;
    theta                = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
    orientation          = sensors.getOrientation();
    theta                = atan2f(
        orientation[1] * orientation[2] + orientation[0] * orientation[3],
        0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]
    );

    delta_theta          = theta - last_theta;

    v                    = delta_s * step_freq;
    w                    = delta_theta * step_freq;

    last_velocity[LEFT]  = wheel_l * step_freq;
    last_velocity[RIGHT] = wheel_r * step_freq;

    // compute odometric pose
    odom_pose[0]         += delta_s * cos(odom_pose[2] + (delta_theta * 0.50));
    odom_pose[1]         += delta_s * sin(odom_pose[2] + (delta_theta * 0.50));
    odom_pose[2]         += delta_theta;

    // compute odometric instantaneouse velocity
    odom_vel[0]          = v;
    odom_vel[1]          = 0.0;
    odom_vel[2]          = w;

    last_theta           = theta;

    return true;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void) 
{
    // Recieve goal velocity through ros messages
    goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
    goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];

}

/*******************************************************************************
* Zetabank
*******************************************************************************/


/*******************************************************************************
* Motor control
*******************************************************************************/
bool controlMotor(const float wheel_separation, float * value) 
{
    
    float wheel_velocity_cmd[2];

    float lin_vel             = value[LINEAR];
    float ang_vel             = value[ANGULAR];

    wheel_velocity_cmd[LEFT]  = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);

        
    writeVelocity((float)wheel_velocity_cmd[LEFT],(float)wheel_velocity_cmd[RIGHT]);

    return true;
}


void motor_setup() 
{

    left_dir_flg  = FORWARD_DIR;
    right_dir_flg = FORWARD_DIR;

    pinMode(6, OUTPUT); //setting left Break
    pinMode(10, OUTPUT); //setting left direction pin CCW
    pinMode(12, OUTPUT); //setting left direction pin CW
    pinMode(0, OUTPUT); //setting right Break
    pinMode(1, OUTPUT); //setting right direction pin CCW
    pinMode(2, OUTPUT); //setting right direction pin CW
}

bool writeVelocity(float left_vel, float right_vel) 
{
    float vel[2] = 
    {
        left_vel,
        right_vel
    };

    for (uint8_t side = LEFT; side <= RIGHT; side++) 
    {
        duty_ratio = pid_control(side, vel[side]);
        motor_pwm(side, duty_ratio);
    }
}

float pid_control(uint8_t side, float ref_vel) 
{
        Kp = 10;

        if (side == LEFT) 
        { // Left wheel
            left_wheel_vel_err = ref_vel - left_wheel_linear_vel;

            if (left_wheel_vel_err > 0.5) 
                Kp = 30;
            else if (left_wheel_vel_err > 0.3) 
                Kp = 25;
            else if (left_wheel_vel_err > 0.1) 
                Kp = 15;
            
            P_control    = Kp * left_wheel_vel_err;
            left_pid_val += P_control;


            //For dead zone
            if (ref_vel > 0 && left_pid_val < 10) 
                left_pid_val = 10;
            else if (ref_vel < 0 && left_pid_val > -10) 
                left_pid_val = -10;
            
            pid_value = left_pid_val;

        } 
        else if (side == RIGHT) 
        { // Right wheel
            right_wheel_vel_err = ref_vel - right_wheel_linear_vel;

            if (right_wheel_vel_err > 0.5) 
                Kp = 30;
            else if (right_wheel_vel_err > 0.3) 
                Kp = 25;
            else if (right_wheel_vel_err > 0.1) 
                Kp = 15;
            
            P_control     = Kp * right_wheel_vel_err;
            right_pid_val += P_control;

            //For dead zone
            if (ref_vel > 0 && right_pid_val < 10) 
                right_pid_val = 10;
            else if (ref_vel < 0 && right_pid_val > -10) 
                right_pid_val = -10;
            
            pid_value = right_pid_val;
        }
 
        if (ref_vel == 0) 
        {
            right_pid_val = 0;
            left_pid_val  = 0;
            
            digitalWrite(0, LOW); // Right break ON
            digitalWrite(6, LOW); // Left break ON
            return 0;
        } 
        else if (-255 <= pid_value && pid_value <= 0) 
        {
            digitalWrite(0, HIGH); // Right break OFF
            digitalWrite(6, HIGH); // Left break OFF
        
            return pid_value;
        }
        else if (255 >= pid_value && pid_value >= 0) 
        {            
          digitalWrite(0, HIGH); // Right break OFF
          digitalWrite(6, HIGH); // Left break OFF
          return pid_value;
        }
        else if (pid_value < -255) 
        {
          return -255;
        }
        else if (pid_value > 255) 
        {
          digitalWrite(0, HIGH); // Right break OFF
          digitalWrite(6, HIGH); // Left break OFF       
          return 255;
        }
}
    
void motor_pwm(uint8_t side, float duty_ratio) 
{
        //BLDC Motor need a 130 value of pwm at least
        if (side == LEFT) 
        { // Left wheel
            if (duty_ratio < 0) 
            { //BACKWARD_DIR
                digitalWrite(10, HIGH); // direction of left CCW
                digitalWrite(12, LOW); // direction of left  CW
                left_dir_flg = BACKWARD_DIR;
            } 
            else if (duty_ratio > 0) 
            { //FORWARD_DIR
                digitalWrite(10, LOW);
                digitalWrite(12, HIGH); // direction of left 
                left_dir_flg = FORWARD_DIR;
            }
            analogWrite(11, abs(duty_ratio)); //first arg : pin number, second arg : duty_ratio. pwm of left
        } 
        else if (side == RIGHT) 
        { // Right wheel
            if (duty_ratio < 0) 
            { //BACKWARD_DIR
                digitalWrite(1, HIGH);  // direction of right CCW
                digitalWrite(2, LOW);  // direction of right CW
                right_dir_flg = BACKWARD_DIR;
            } 
            else if (duty_ratio > 0) 
            { //FORWARD_DIR
                digitalWrite(1, LOW);  // direction of right CCW
                digitalWrite(2, HIGH);  // direction of right CW
                right_dir_flg = FORWARD_DIR;
            }
            analogWrite(9, abs(duty_ratio)); //first arg : pin number, second arg : duty_ratio. pwm of right
        }
        delay(10);
}


/*******************************************************************************
* Setting for encoder
*******************************************************************************/
void timer_setup() 
{
    Timer.stop();
    Timer.setPeriod(100000); // in microseconds, 100000us -> 100ms
    Timer.attachInterrupt(timerInterrupt);
    Timer.start();    
}

void encoder_setup() 
{
    /* 0: PIN 2, EXTI_0
    1: PIN 3, EXTI_1
    2: PIN 4, EXTI_2
    3: PIN 7, EXTI_3
    4: PIN 8, EXTI_4
    */
    enc_cnt                  = 0;

    n_left_enc_A_pulse       = 0;
    n_left_enc_B_pulse       = 0;
    n_left_enc_A_pulse_mean  = 0;
    n_left_enc_B_pulse_mean  = 0;
    n_right_enc_A_pulse      = 0;
    n_right_enc_B_pulse      = 0;
    n_right_enc_A_pulse_mean = 0;
    n_right_enc_B_pulse_mean = 0;

    n_left_odom_A_cnt        = 0;
    n_left_odom_B_cnt        = 0;
    n_right_odom_A_cnt       = 0;
    n_right_odom_B_cnt       = 0;

    pinMode(3, INPUT_PULLDOWN); //right encoder A
    pinMode(4, INPUT_PULLDOWN); //right encoder B
    pinMode(7, INPUT_PULLDOWN); //left encoder A
    pinMode(8, INPUT_PULLDOWN); //left encoder B

    attachInterrupt(3, left_EncoderRead_A, RISING); //left encoder A, pin 7
    attachInterrupt(4, left_EncoderRead_B, RISING); //left encoder B, pin 8
    attachInterrupt(1, right_EncoderRead_A, RISING); //right encoder A, pin 3
    attachInterrupt(2, right_EncoderRead_B, RISING); //right encoder B, pin 4

}

void left_EncoderRead_A() 
{
    n_left_enc_A_pulse++;
    n_left_odom_A_cnt++;
   
}

void left_EncoderRead_B() 
{
    n_left_enc_B_pulse++;
     n_left_odom_B_cnt++;
}

void right_EncoderRead_A() 
{
    n_right_enc_A_pulse++;
    n_right_odom_A_cnt++;
}

void right_EncoderRead_B() 
{
    n_right_enc_B_pulse++;
    n_right_odom_B_cnt++;
}
    
void timerInterrupt(void) 
{
    /* 
    Serial.print("Angular Velocity :");
    Serial.println(n_enc_pulse * 0.2929 ); //[deg/sec]
    //1024 * 12 cnt = 1 round, 1 cnt = (360 / 12288) = 0.02929 deg
    wheel_linear_vel = n_enc_pulse * 0.0004151; // 1 cnt [ (0.02929 * radius * 3.1415/2pi ) / 0.1sec] * = [m/s]

    Serial.print("Linear Velocity :");
    Serial.println(wheegl_linear_vel); // [meter/sec]
    v = R*w = 0.0812 [m] * ( n_pulse * 0.2929 ) [deg/sec] * ( 2 PI / 360 )[rad/deg]

    // NSCLbot
    //1000 * 26 cnt = 1 round, 1 cnt = (360 / 26000) = 0.013846154 deg
    v = R*w * 10(100msec->1sec) = 0.127 [m] * ( n_pulse * 0.013846154  ) [deg/sec] * ( 2 PI / 360 )[rad/deg] * 10= 0.00030691
    */
    enc_cnt++;

    n_right_enc_A_pulse_mean += n_right_enc_A_pulse;
    n_right_enc_B_pulse_mean += n_right_enc_B_pulse;
    n_left_enc_A_pulse_mean  += n_left_enc_A_pulse;
    n_left_enc_B_pulse_mean  += n_left_enc_B_pulse;

    if(enc_cnt >= 1) 
    {
        //100ms 간격으로 속도 update, n번마다 velocity update, mean /2.
        if (left_dir_flg == FORWARD_DIR) 
            left_wheel_linear_vel = (n_left_enc_A_pulse_mean + n_left_enc_B_pulse_mean) * 0.5 * 0.00030691;
        else if (left_dir_flg == BACKWARD_DIR) 
            left_wheel_linear_vel = -((n_left_enc_A_pulse_mean + n_left_enc_B_pulse_mean) * 0.5 * 0.000306917);
            
        if (right_dir_flg == FORWARD_DIR) 
            right_wheel_linear_vel = (n_right_enc_A_pulse_mean + n_right_enc_B_pulse_mean) * 0.5 * 0.00030691;
        else if (right_dir_flg == BACKWARD_DIR) 
            right_wheel_linear_vel = -((n_right_enc_A_pulse_mean + n_right_enc_B_pulse_mean) * 0.5 * 0.00030691);

            
            right_vel_msg.data = right_wheel_linear_vel;
            right_vel_pub.publish(& right_vel_msg);
            left_vel_msg.data = left_wheel_linear_vel;
            left_vel_pub.publish(& left_vel_msg);

            n_right_enc_A_pulse_mean = 0;
            n_right_enc_B_pulse_mean = 0;
            n_left_enc_A_pulse_mean  = 0;
            n_left_enc_B_pulse_mean  = 0;
            enc_cnt                  = 0;
    }

    n_right_enc_A_pulse = 0;
    n_right_enc_B_pulse = 0;
    n_left_enc_A_pulse  = 0;
    n_left_enc_B_pulse  = 0;
}


/*******************************************************************************
* Setting for Ultrasonic sensor
*******************************************************************************/
void ultrasonic_setup() 
{
  pinMode(BDPIN_GPIO_17, OUTPUT);
  pinMode(BDPIN_GPIO_18, INPUT);

  digitalWrite(BDPIN_GPIO_17, HIGH ); //start bit
  delay(1);
  digitalWrite(BDPIN_GPIO_17, LOW );
  delayMicroseconds(200);
  digitalWrite(BDPIN_GPIO_17, HIGH );
  delayMicroseconds(100);
  digitalWrite(BDPIN_GPIO_17, LOW );
  delayMicroseconds(200);
  digitalWrite(BDPIN_GPIO_17, HIGH );
  delayMicroseconds(100);
  digitalWrite(BDPIN_GPIO_17, LOW );
  delayMicroseconds(300);
  digitalWrite(BDPIN_GPIO_17, HIGH );
  
  delay(1);
  digitalWrite(BDPIN_GPIO_17, LOW );

}

void ultrasonic_check()
{
    
  if(digitalRead(BDPIN_GPIO_18) == HIGH)  //input GPIO 20
  {
    ultra_flg = false; // Not detect
  }
  else if(digitalRead(BDPIN_GPIO_18) == LOW)
  {
    ultra_flg = true; //Detect
  }
    
  sonar_msg.header.stamp    = rosNow();
  sonar_msg.header.frame_id = "sonar_link";
  
  sonar_msg.radiation_type = 0; 
  
  sonar_msg.field_of_view = 0.436332222;  
  sonar_msg.min_range = 0.01;  
  sonar_msg.max_range = 0.500;
    
  if( ultra_flg == true)
    sonar_msg.range = 0.4;  
  else if( ultra_flg == false)
    sonar_msg.range = 0.0; 
     
  sonar_pub.publish(&sonar_msg);
}


/*******************************************************************************
* Network disconnection for protecting something
*******************************************************************************/

void network_disconnect()
{
    // Stop the motor
    resetGoalVelocity();
    controlMotor(WHEEL_SEPARATION, goal_velocity);
    
    // Check the obstacle using Ultrasonic sensors
    //ultrasonic_check();

}

void resetGoalVelocity()
{
    // Reset the goal velocities
    goal_velocity[LINEAR]  = 0.0f;
    goal_velocity[ANGULAR] = 0.0f;
    goal_velocity_from_cmd[LINEAR]  = 0.0f;
    goal_velocity_from_cmd[ANGULAR] = 0.0f;

}
    
