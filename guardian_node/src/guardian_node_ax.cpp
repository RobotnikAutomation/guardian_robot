/*
 * guardian_node
 * Copyright (c) 2011, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation SLL
 */

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <guardian_node/ax3500.h>
#include <guardian_node/guardian_state.h>
#include <guardian_node/set_odometry.h>
#include <guardian_node/config_pid.h>
#include <guardian_node/set_pid.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <string>
#include <ros/ros.h>
#include <boost/assign.hpp>

#define		GUARDIAN_DEFAULT_MAX_LINEAR_SPEED	1.5		// m/s
#define		GUARDIAN_DEFAULT_MAX_ANGULAR_SPEED	40.0	// degrees/s
#define		GUARDIAN_DEFAULT_ODOMETRY			ODOMTYPE_WHEELS

#define		GUARDIAN_MIN_COMMAND_REC_FREQ	1.0
#define		GUARDIAN_MAX_COMMAND_REC_FREQ	20.0


//////////////////////////
// Variable definitions //
//////////////////////////

std::string port;

ax3500* guardian_hw_interface; 						// Instance to the interface
//modbus_io::modbus_io_msg inputs_outputs_value;		// variable with the I/O values read from the modbus_io node
guardian_node::guardian_state robot_state;			// Guardian's state message
double v, w = 0.0; 									// Callback cmd vel variables
std::string sOdometryType_; 						// Type of odometry used by the robot
double dt = 0.0;									// Odometry variables
diagnostic_updater::HeaderlessTopicDiagnostic *sus_command_freq, *sus_io_freq; // Diagnostic to control the frequency of topics

ros::Time last_command_time;					// Last moment when the component received a command

// Recovery attemps control
int maxAttemps = 10000;
int attemps = 0;

// IMU values
double ang_vel_x_ = 0.0;
double ang_vel_y_ = 0.0;
double ang_vel_z_ = 0.0;

double lin_acc_x_ = 0.0;
double lin_acc_y_ = 0.0;
double lin_acc_z_ = 0.0;

double orientation_x_ = 0.0;
double orientation_y_ = 0.0;
double orientation_z_ = 0.0;
double orientation_w_ = 0.0;


//////////////////////////
// Callback definitions //
//////////////////////////

/*
 * \brief Function called when a motor command is received
 */
void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
	// Multiplied * -1 to adapt the real direction with the joystick commands
	v = cmd_vel->linear.x;// * -1;
	w = cmd_vel->angular.z;// * -1;					
	guardian_hw_interface->SetSpeed(v, w);
	sus_command_freq->tick();			// For diagnostics
    last_command_time = ros::Time::now();
	//ROS_ERROR("Guardian Node: Command vel received v= %f, w = %f", v, w);
}



void imuCallback(const sensor_msgs::Imu& imu_msg){
	orientation_x_ = imu_msg.orientation.x;
	orientation_y_ = imu_msg.orientation.y;
	orientation_z_ = imu_msg.orientation.z;
	orientation_w_ = imu_msg.orientation.w;

	ang_vel_x_ = imu_msg.angular_velocity.x;
	ang_vel_y_ = imu_msg.angular_velocity.y;
	ang_vel_z_ = imu_msg.angular_velocity.z;

	lin_acc_x_ = imu_msg.linear_acceleration.x;
	lin_acc_y_ = imu_msg.linear_acceleration.y;
	lin_acc_z_ = imu_msg.linear_acceleration.z;
}

// 
/*void ioCallback(const modbus_io::modbus_io_msg::ConstPtr& io_msg){
	//ROS_INFO("ioCallback");
	inputs_outputs_value.digital_input = io_msg->digital_input;
	inputs_outputs_value.digital_output = io_msg->digital_output;
	inputs_outputs_value.analog_input = io_msg->analog_input;
	inputs_outputs_value.analog_input_cfg = io_msg->analog_input_cfg;
	inputs_outputs_value.analog_output = io_msg->analog_output;
		
}*/

/* 
 * Converts radians to degrees
*/
double RTOD(double val){
    return val*180.0/Pi;
}

/* 
 * Converts degrees to radians
 */
double DTOR(double val){
    return val*Pi/180.0;
}

/* 
 * Check the robot's state and modifies the guardian_state variable
 */
void CheckRobotState(){
	ax3500_data ax_data;
	pose ax_pose;
		
	//States state = guardian_hw_interface->GetState();
	robot_state.status_msg.assign("--");
	//
	ax_data = guardian_hw_interface->GetData();		// Gets other data
	ax_pose = guardian_hw_interface->GetPose(); 	// Gets odometry data
	
	/*if(inputs_outputs_value.digital_input[0]){
		robot_state.status_msg.assign("Emergency stop ACTIVE!");
	}else if(guardian_hw_interface->GetState() == FAILURE_STATE ){
		robot_state.status_msg.assign("Error in communication with the motor controller");
	}*/
	robot_state.state.assign(guardian_hw_interface->GetStateString());	// State
	
	robot_state.battery_voltage = guardian_hw_interface->GetVoltage();	// Voltage
	robot_state.battery = guardian_hw_interface->GetBatteryPercent();	// Voltage %
	robot_state.temp[0] = ax_data.temperature[0];						// Temp 1
	robot_state.temp[1] = ax_data.temperature[1];						// Temp 2
	robot_state.channel_ref[0] = ax_data.channel_A_ref;					// Channel A
	robot_state.channel_ref[1] = ax_data.channel_B_ref;					// Channel B
	robot_state.motor_control_mode.assign(guardian_hw_interface->GetMotorControlModeString());	// Motor control mode
	robot_state.motor_input_mode.assign(guardian_hw_interface->GetInputControlModeString());	// Motor input mode
	guardian_hw_interface->GetDesiredSpeed(&robot_state.desired_speed[0], &robot_state.desired_speed[1]); // Desired speeds (linear, angular)	
	robot_state.real_speed[0] = sqrt(ax_pose.vx*ax_pose.vx + ax_pose.vy*ax_pose.vy);	// linear speed (m/s)
	robot_state.real_speed[1] = ax_pose.va;	// angular speed (rad/s)
	robot_state.rpm_motors[0] = ax_data.rpm_left;		// left motor
	robot_state.rpm_motors[1] = ax_data.rpm_right;		// right motor
	robot_state.controller_hz = guardian_hw_interface->GetUpdateRate();	// Motor controller update rate
	robot_state.odometry_type = sOdometryType_;	// Type of odometry (TRACK or WHEELS)
	robot_state.internal_odometry[0] = ax_pose.px;	// For test purposes
	robot_state.internal_odometry[1] = ax_pose.py;	// For test purposes
	robot_state.internal_odometry[2] = ax_pose.pa;	// For test purposes
	robot_state.current_consumption[0] = ax_data.current_consumption[0];	// Current motor 1
	robot_state.current_consumption[1] = ax_data.current_consumption[1];	// Current motor 2
	
}

/*
*  Service set odometry
*  Sets the odometry of the robot
 */
bool set_odometry(guardian_node::set_odometry::Request &req,
         guardian_node::set_odometry::Response &res )
{
	ROS_INFO("guardian_node::set_odometry: request -> x = %f, y = %f, a = %f", req.x, req.y, req.orientation);

	guardian_hw_interface->ModifyOdometry(req.x, req.y, req.orientation);
	res.ret = true;
	
	return true;
}

/* 
 * Gets the current PID values 
*/
bool get_pid(guardian_node::config_pid::Request &req,
         guardian_node::config_pid::Response &res )
{
	
	ROS_INFO("guardian_node::get_pid");
	pid_params pid;
         
	pid = guardian_hw_interface->GetPID();

	res.p_linear_divisor = (float) pid.KpL_divisor;
	res.p_angular_divisor = (float) pid.KpA_divisor;
	res.i_linear  = (float)pid.KiL;
	res.i_angular = (float) pid.KiA;
	res.i_error_sat_linear = (float) pid.ErrSatIL_divisor;
	res.i_error_sat_angular = (float) pid.ErrSatIA_divisor;

	return true;
}

/* 
 * Gets the current PID values 
*/
bool set_pid(guardian_node::set_pid::Request &req,
         guardian_node::set_pid::Response &res )
{
	
	ROS_INFO("guardian_node::set_pid");
	pid_params pid;

	pid.KpL_divisor = (float)req.p_linear_divisor;
	pid.KpA_divisor = (float) req.p_angular_divisor;
	pid.KiL = (float) req.i_linear;
	pid.KiA = (float) req.i_angular;
	pid.ErrSatIL_divisor = (float)req.i_error_sat_linear;
	pid.ErrSatIA_divisor = (float)req.i_error_sat_angular;

	guardian_hw_interface->SetPID(pid);

	res.ret = true;
	
	return true;
}

/*
 *\brief Checks the status of the controller. Diagnostics
 *
 */
void controller_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	States state = guardian_hw_interface->GetState();
	ax3500_data ax_data  = guardian_hw_interface->GetData();

	if(state == READY_STATE){
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Controller ready");
	}else if(state == INIT_STATE){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Controller initializing");
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Controller on Failure or unknown state");
	}
	
	// add and addf are used to append key-value pairs.
	stat.add("State", guardian_hw_interface->GetStateString()); // Internal controller state
	
	stat.add("Controller temp 1", ax_data.temperature[0]); // Temp 1
	stat.add("Controller temp 2", ax_data.temperature[1]); // Temp 2
	stat.add("Control Mode", guardian_hw_interface->GetMotorControlModeString()); // Control mode
	stat.add("Input Mode", guardian_hw_interface->GetInputControlModeString()); // Input mode
	stat.add("Odometry type", sOdometryType_.c_str()); // Odometry type

}

/*
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 *
 */
void check_command_suscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time).toSec();

	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
		//ROS_INFO("check_command_suscriber: %lf seconds without commands", diff);
		guardian_hw_interface->SetSpeed(0.0, 0.0);
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
}

/*
 *	\brief Checks the battery & power supply. Diagnostics
 *
 */
void check_powersupply(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	double batt = guardian_hw_interface->GetBatteryPercent();	// Voltage %
	double volt = guardian_hw_interface->GetVoltage();	// Voltage

	if(guardian_hw_interface->GetState() != READY_STATE){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No communication with the controller");
	}else{
	
		if(batt > 60.0){
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Good battery level");
		}else if( (batt < 60.0) && (batt > 30.0)){
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low battery Level.");
		}else{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Extremely low battery Level. You should charge the battery.");
		}

		stat.add("Battery Voltage", volt); // Battery Voltage
		stat.addf("Battery (%)", "%.3f", batt); // Battery %
	}
}

//////////
// main //
//////////
int main(int argc, char** argv){

	ros::init(argc, argv, "guardian_node");
	ros::NodeHandle n;
	pose robot_pose;	// Position & velocity from the odometry
	double max_linear_speed_, max_angular_speed_;
	double diameter_wheels_, distance_between_wheels_;	// Diameter of the wheels and distance between them
	std::string sDefaultOdometryType(GUARDIAN_DEFAULT_ODOMETRY);
	double pid_p_l_divisor_,  pid_p_a_divisor_, pid_i_l_, pid_i_a_, pid_err_sat_l_divisor_, pid_err_sat_a_divisor_;	// PID constants
	int encoder_config_, encoder_dir_, angular_dir_;
	bool publish_tf_ = false;
	//
	// The Updater class advertises to /diagnostics, and has a
	// ~diagnostic_period parameter that says how often the diagnostics
	// should be published. 
	diagnostic_updater::Updater updater_controller;	// General status diagnostic updater
	updater_controller.setHardwareID("guardian_controller-AX");
	updater_controller.add("Motor Controller", controller_diagnostic);

	diagnostic_updater::FunctionDiagnosticTask command_freq("Command frequency check",
      boost::bind(&check_command_suscriber, _1));

	diagnostic_updater::FunctionDiagnosticTask battery_control("Power system",
      boost::bind(&check_powersupply, _1));
	
	updater_controller.add(battery_control);		// Adding the battery control task to the diagnostic updater
	
	// Name of the topic where the I/O values are published
	//std::string modbus_io_topic_;
	//std::string sDefaultIOtopic("/modbus_io/input_output");
		
	// set the motor controller's device port, reading the configuration
	std::string sDevicePort;
	std::string sDefaultPort(AX3500_DEFAULT_PORT);
	
	
	ROS_INFO("Running Guardian ROS.");
	// READING PARAMETERS
	//n.param("modbus_io_topic", modbus_io_topic_,  sDefaultIOtopic);
	n.param("motor_dev", sDevicePort,  sDefaultPort);
	n.param("max_linear_speed", max_linear_speed_,  GUARDIAN_DEFAULT_MAX_LINEAR_SPEED);
	n.param("max_angular_speed", max_angular_speed_,  GUARDIAN_DEFAULT_MAX_ANGULAR_SPEED);
	n.param("odometry_type", sOdometryType_, sDefaultOdometryType);
	n.param("diameter_wheels", diameter_wheels_,  MOTOR_DIAMETER_WHEEL);
	n.param("distance_between_wheels", distance_between_wheels_,  MOTOR_D_WHEELS_M_GWAM);
	n.param("pid_p_l_divisor", pid_p_l_divisor_,  AX3500_DEFAULT_P_L_DIVISOR);
	n.param("pid_p_a_divisor", pid_p_a_divisor_,  AX3500_DEFAULT_P_A_DIVISOR);
	n.param("pid_i_l", pid_i_l_,  AX3500_DEFAULT_I_L);
	n.param("pid_i_a", pid_i_a_,  AX3500_DEFAULT_I_A);
	n.param("pid_err_sat_l_divisor", pid_err_sat_l_divisor_,  AX3500_DEFAULT_ERR_SAT_L_DIVISOR);
	n.param("pid_err_sat_a_divisor", pid_err_sat_a_divisor_,  AX3500_DEFAULT_ERR_SAT_A_DIVISOR);
	n.param("encoder_config", encoder_config_, AX3500_ENCODER_CONF);
	n.param("encoder_dir", encoder_dir_, AX3500_ENCODER_DIR);
	n.param("angular_dir", angular_dir_, AX3500_ANGULAR_DIR);
	n.param("publish_tf", publish_tf_, publish_tf_);
	
	        
	////// PRINTING INFORMATION
	ROS_INFO("guardian_node::main: Motor dev = %s", sDevicePort.c_str());
	ROS_INFO("guardian_node::main: Max Speed: v = %.2f m/s, w = %.2f d/s", max_linear_speed_, max_angular_speed_);
	ROS_INFO("guardian_node::main: Odometry type: %s", sOdometryType_.c_str());
	ROS_INFO("guardian_node::main: diameter_wheels: v = %.3f m", diameter_wheels_);
	ROS_INFO("guardian_node::main: encoder_config = %d, encoder_dir = %d, angular_dir = %d", encoder_config_, encoder_dir_, angular_dir_);
	ROS_INFO("guardian_node::main: PID P divisors: L = %.3f, A = %.3f", pid_p_l_divisor_, pid_p_a_divisor_);
	ROS_INFO("guardian_node::main: PID I: L = %.3f, A = %.3f", pid_i_l_, pid_i_a_);
	ROS_INFO("guardian_node::main: PID I error sat divisor: L = %.3f, A = %.3f", pid_err_sat_l_divisor_, pid_err_sat_a_divisor_); 
	
	// Motor controller interface creation (Closed Loop Mixed Velocity Control)
	guardian_hw_interface = new ax3500((char*)sDevicePort.c_str(), 15.0);									
	// Applying encoder config...
	guardian_hw_interface->SetEncoderConfig(encoder_config_, encoder_dir_, angular_dir_);
  	
	// Applies the configuration to the driver
	// For odom using tracks, the configuration is pre-defined by default
	if(!sOdometryType_.compare(ODOMTYPE_TRACKS)){
		distance_between_wheels_ = MOTOR_D_TRACKS_M_GWAM;
		distance_between_wheels_ *= ERROR_D_2;	// Applying error factor for odom calcs
		diameter_wheels_ = MOTOR_DIAMETER_TRACK_WHEEL;	
		guardian_hw_interface->SetConversionFactor(MOTOR_GEARBOX_TO_TRACK_FACTOR);
	}else{
		distance_between_wheels_ *= ERROR_D_1;	// Applying error factor for odom calcs
	}
	
	// Sets the max. speed read from the param
	guardian_hw_interface->SetSpeedLimits(max_linear_speed_, DTOR(max_angular_speed_));
	guardian_hw_interface->SetP(pid_p_l_divisor_, pid_p_a_divisor_);
	guardian_hw_interface->SetI(pid_i_l_, pid_i_a_, pid_err_sat_l_divisor_, pid_err_sat_a_divisor_);
	guardian_hw_interface->SetMotorWheelParams(diameter_wheels_, distance_between_wheels_);
	guardian_hw_interface->EnablePID(false);
	guardian_hw_interface->ToggleMotorPower(true);
	guardian_hw_interface->SetSpeed(0.0, 0.0);

	
	//
	// Setups and starts the component
	if(guardian_hw_interface->Setup()!= OK) {
	    ROS_ERROR("main:Error in guardian Setup");
	}else if(guardian_hw_interface->Start()!= OK){
	    ROS_ERROR("main: Error in guardian Start");
	}
	//
	// Defining subscribers to obtain information through the sensors and joysticks
  	ros::Subscriber cmd_sub_ = n.subscribe<geometry_msgs::Twist>("/guardian_node/command", 1, cmdCallback); 
  	ros::Subscriber imu_sub_ = n.subscribe("/imu/data", 1, imuCallback);
	//ros::Subscriber io_sub_ = n.subscribe(modbus_io_topic_, 1, ioCallback);
	//
	// Defining publishers
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/guardian_node/odom", 30);
	ros::Publisher state_pub = n.advertise<guardian_node::guardian_state>("/guardian_node/state", 30);
	tf::TransformBroadcaster odom_broadcaster;
	//
	// Defining services

	ros::ServiceServer set_odometry_srv_ = n.advertiseService("/guardian_node/set_odometry",  set_odometry);
	ros::ServiceServer get_pid_srv_ = n.advertiseService("/guardian_node/get_pid",  get_pid);
	ros::ServiceServer set_pid_srv_ = n.advertiseService("/guardian_node/set_pid",  set_pid);
	
	
	// Topics freq control 
	// For /guardian_node/command
	double min_freq = GUARDIAN_MIN_COMMAND_REC_FREQ; // If you update these values, the
  	double max_freq = GUARDIAN_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
	sus_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/guardian_node/command", updater_controller,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
	sus_command_freq->addTask(&command_freq); // Adding an additional task to the control
	//
	// For IO topic 
	double min_io_freq = 10.0; // If you update these values, the
  	double max_io_freq = 50.0; // HeaderlessTopicDiagnostic will use the new values.
	//sus_io_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(modbus_io_topic_, updater_controller,
	  //                  diagnostic_updater::FrequencyStatusParam(&min_io_freq, &max_io_freq, 0.1, 10));

	
	ros::Time current_time, last_time;
	// Initializes timers
	last_command_time = current_time = last_time = ros::Time::now();
	
	ros::Rate r(15.0);  // 50.0 
	
	//while( attemps < maxAttemps ) {

	while( ros::ok() ){ // n.ok() && (guardian->GetCurrentState() == 1)

		current_time = ros::Time::now();	

		// Obtain the odometry 
		robot_pose = guardian_hw_interface->GetPose();;

		//ROS_WARN("*********************************\n");
		//ROS_INFO("Vx: %d Vy: %d Va: %d\n", vx, vy, va);
		//ROS_INFO("Px: %d Py: %d Pa: %d\n", px, py, pa);
		//ROS_INFO("Temp: %d\n", guardian->GetTemp(1)) ;
		//ROS_INFO("Voltage: %d\n", guardian->GetVoltage());
		//ROS_INFO("GetEncoder(L): %f\n", guardian->GetEncoder('L') );
		//ROS_INFO("GetEncoder(R): %f\n", guardian->GetEncoder('R') );
		//ROS_WARN("*********************************\n");

		dt = (current_time - last_time).toSec();

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.pa);

		if(publish_tf_){
			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_footprint"; // base_link

			odom_trans.transform.translation.x = robot_pose.px;
			odom_trans.transform.translation.y = robot_pose.py;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);
		}
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint"; // base_link
		
		//set the position
		odom.pose.pose.position.x = robot_pose.px;
		odom.pose.pose.position.y = robot_pose.py;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		// TEST: Adding static covariance
		odom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3) ;

		//double yfq = tf::getYaw(odom_quat);
		//ROS_ERROR("EULER YAW FROM QUATERNION:%f", yfq);

		//set the velocity		
		odom.twist.twist.linear.x = robot_pose.vx;
		odom.twist.twist.linear.y = robot_pose.vy;
		odom.twist.twist.angular.z = robot_pose.va;
		// TEST: Adding static covariance
		odom.twist.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3) ;

		//
		//publish the messages
		odom_pub.publish(odom);			// Odometry
		CheckRobotState();
		state_pub.publish(robot_state);	// State
		last_time = current_time;

		// UPDATING DIAGNOSTICS
		updater_controller.update();
			
		ros::spinOnce();
		r.sleep();
	}
		
	//	ROS_WARN("guardian_node::main bucle: Trying to recover the ready state.");

	//	attemps = attemps + 1;

	//}

	ROS_ERROR("guardian_node::main: ending program");
	
	guardian_hw_interface->SetSpeed(0, 0);
	guardian_hw_interface->ToggleMotorPower(false);
	guardian_hw_interface->Stop();

	delete guardian_hw_interface;
}


