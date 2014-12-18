/** \file guardian_node.cc
 * \brief guardian_node for ROS
 * 		Node to manage the communication with the motors and publish the odometry
 * 		Version for NTXGEN controller
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
 * \author Robotnik Automation
 *		   Román Navarro(rnavarro@robotnik.es)
 * (C) 2012 Robotnik Automation, SLL
 */

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <guardian_node/guardian_controller.h>
#include <guardian_node/guardian_state.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <guardian_node/set_odometry.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <boost/assign.hpp>

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#define		GUARDIAN_DEFAULT_MAX_LINEAR_SPEED	1.5		// m/s
#define		GUARDIAN_DEFAULT_MAX_ANGULAR_SPEED	40.0	// degrees/s
#define		GUARDIAN_DEFAULT_ODOMETRY			ODOMTYPE_WHEELS

#define 	ODOMTYPE_WHEELS						"wheels"		
#define 	ODOMTYPE_TRACKS						"tracks"

#define		GUARDIAN_MIN_COMMAND_REC_FREQ	1.0
#define		GUARDIAN_MAX_COMMAND_REC_FREQ	20.0

#define 	GUARDIAN_JOINTS	4		// Number of joints to publish

//////////////////////////
// Variable definitions //
//////////////////////////

std::string port;

guardian_controller* guardian_hw_interface; 		// Instance to the interface
guardian_node::guardian_state robot_state;			// Guardian's state message
double v, w = 0.0; 									// Callback cmd vel variables
std::string sOdometryType_; 						// Type of odometry used by the robot
double dt = 0.0;									// Odometry variables
ros::Publisher joint_state_pub_;					// Joint State publisher


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


diagnostic_updater::HeaderlessTopicDiagnostic *sus_command_freq, *sus_io_freq; // Diagnostic to control the frequency of topics

ros::Time last_command_time;					// Last moment when the component received a command


//////////////////////////
// Callback definitions //
//////////////////////////
void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
	v = cmd_vel->linear.x;// * -1;
	w = cmd_vel->angular.z;// * -1;					
	guardian_hw_interface->SetSpeed(v, w);
	sus_command_freq->tick();			// For diagnostics
    last_command_time = ros::Time::now();
//	ROS_INFO("Guardian Node: Command vel received v= %f, w = %f", v, w);
}



//! Converts radians to degrees
double RTOD(double val){
    return val*180.0/Pi;
}

//! Converts degrees to radians
double DTOR(double val){
    return val*Pi/180.0;
}

//! Check the robot's state and modifies the guardian_state variable
void CheckRobotState(){
	//States state = guardian_hw_interface->GetState();
	robot_state.status_msg.assign("--");

	roboteq_data ax_data;
	pose ax_pose;
		
	//States state = guardian_hw_interface->GetState();
	robot_state.status_msg.assign("--");
	//
	ax_data = guardian_hw_interface->GetData();		// Gets other data
	ax_pose = guardian_hw_interface->GetPose(); 	// Gets odometry data
	
	
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
 *\brief Checks the status of the controller. Diagnostics
 *
 */
void controller_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	States state = guardian_hw_interface->GetState();
	roboteq_data ax_data  = guardian_hw_interface->GetData();

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
			/*std::cout << RED <<"**************************************************" << RESET << std::endl;
			std::cout << RED <<"********** Extremely low battery Level: " << batt << "**********" << RESET << std::endl;
			std::cout << RED <<"**************************************************" << RESET << std::endl;*/		
		}

		stat.add("Battery Voltage", volt); // Battery Voltage
		stat.addf("Battery (%)", "%.3f", batt); // Battery %

		 std::cout << GREEN <<"*** battery: " << batt  << RESET;
		        std::cout << RED <<"%| v:  " << volt << RESET << std::endl;

	}
}

//////////
// main //
//////////
int main(int argc, char** argv){
	int t = 5;
	ros::init(argc, argv, "guardian_node");
	ros::NodeHandle n, pn("~");
	pose robot_pose;	// Position & velocity from the odometry
	double max_linear_speed_, max_angular_speed_;
	double diameter_wheels_, distance_between_wheels_, error_factor_;	// Diameter of the wheels and distance between them
	sensor_msgs::JointState robot_joints_;
	double desired_freq_ = 0;
	
	std::string sDefaultOdometryType(GUARDIAN_DEFAULT_ODOMETRY);
	int encoder_config_, encoder_dir_, angular_dir_;
	bool publish_tf_ = false;
	float left_rpm = 0.0, right_rpm = 0.0;
	double inc_left_wheel = 0.0, inc_right_wheel = 0.0;
	std::string back_left_wheel_joint_name_, back_right_wheel_joint_name_, front_left_wheel_joint_name_, front_right_wheel_joint_name_;
	bool invert_odom_tf_;
	//
	// The Updater class advertises to /diagnostics, and has a
	// ~diagnostic_period parameter that says how often the diagnostics
	// should be published. 
	diagnostic_updater::Updater updater_controller;	// General status diagnostic updater
	updater_controller.setHardwareID("guardian_controller-NTXGEN");
	updater_controller.add("Motor Controller", controller_diagnostic);

	diagnostic_updater::FunctionDiagnosticTask command_freq("Command frequency check",
      boost::bind(&check_command_suscriber, _1));

	diagnostic_updater::FunctionDiagnosticTask battery_control("Power system",
      boost::bind(&check_powersupply, _1));
	
	updater_controller.add(battery_control);		// Adding the battery control task to the diagnostic updater
	
	//! Name of the topic where the I/O values are published
	//std::string modbus_io_topic_;
	//std::string sDefaultIOtopic("/modbus_io/input_output");
		
	// set the device port, reading the configuration
	std::string sDevicePort;
	std::string sDefaultPort(GUARDIAN_CONTROLLER_DEFAULT_PORT);
	
	
	ROS_INFO("Running Guardian ROS.");
	// READING PARAMETERS
	//pn.param("modbus_io_topic", modbus_io_topic_,  sDefaultIOtopic);
	pn.param("motor_dev", sDevicePort,  sDefaultPort);
	pn.param("max_linear_speed", max_linear_speed_,  GUARDIAN_DEFAULT_MAX_LINEAR_SPEED);
	pn.param("max_angular_speed", max_angular_speed_,  GUARDIAN_DEFAULT_MAX_ANGULAR_SPEED);
	pn.param("odometry_type", sOdometryType_, sDefaultOdometryType);

    if (sOdometryType_ == "tracks")
    {
        pn.param("diameter_wheels", diameter_wheels_,  MOTOR_DIAMETER_TRACK_WHEEL);
        pn.param("distance_between_wheels", distance_between_wheels_,  MOTOR_D_TRACKS_M);
        pn.param("error_factor", error_factor_, ERROR_D_2);
    }
    else
    {
        pn.param("diameter_wheels", diameter_wheels_,  MOTOR_DIAMETER_WHEEL);
        pn.param("distance_between_wheels", distance_between_wheels_,  MOTOR_D_WHEELS_M);
        pn.param("error_factor", error_factor_, ERROR_D_1);
    }
	pn.param("encoder_config", encoder_config_, ROBOTEQ_DEFAULT_ENCODER_CONF);
	pn.param("encoder_dir", encoder_dir_, ROBOTEQ_DEFAULT_ENCODER_DIR);
	pn.param("angular_dir", angular_dir_, ROBOTEQ_DEFAULT_ANGULAR_DIR);
    pn.param("publish_tf", publish_tf_, true);
	pn.param("desired_freq", desired_freq_, 20.0);
	pn.param<std::string>("back_left_wheel_joint_name", back_left_wheel_joint_name_, "joint_back_left_wheel");
	pn.param<std::string>("back_right_wheel_joint_name", back_right_wheel_joint_name_, "joint_back_right_wheel");
	pn.param<std::string>("front_left_wheel_joint_name", front_left_wheel_joint_name_, "joint_front_left_wheel");
	pn.param<std::string>("front_right_wheel_joint_name", front_right_wheel_joint_name_, "joint_front_right_wheel");
	
	pn.param("invert_odom_tf", invert_odom_tf_, false);

	ROS_INFO("guardian_node::main: Motor dev = %s", sDevicePort.c_str());
	ROS_INFO("guardian_node::main: Max Speed: v = %.2f m/s, w = %.2f d/s", max_linear_speed_, max_angular_speed_);
	ROS_INFO("guardian_node::main: Odometry type: %s", sOdometryType_.c_str());
	ROS_INFO("guardian_node::main: diameter_wheels: %.3f m", diameter_wheels_);
	ROS_INFO("guardian_node::main: Distance between wheels:  %.3f m", distance_between_wheels_);
	ROS_INFO("guardian_node::main: Encoder config: %d, dir: %d, angular_dir = %d", encoder_config_, encoder_dir_, angular_dir_);
	ROS_INFO("guardian_node::main: Publish TF:%s", publish_tf_?"true":"false");
	// Interface creation (Closed Loop Mixed Velocity Control)
	guardian_hw_interface = new guardian_controller((char*)sDevicePort.c_str(), 20.0);		
	// Applying encoder config...
	guardian_hw_interface->SetEncoderConfig(encoder_config_, encoder_dir_, angular_dir_);

  	if(!guardian_hw_interface){
      	ROS_ERROR("Something wrong with the hardware interface pointer!");
  	}else{
		// Applies the configuration to the driver
		// For odom using tracks, the configuration is pre-defined by default
		/*if(!sOdometryType_.compare(ODOMTYPE_TRACKS)){
			distance_between_wheels_ *= ERROR_D_2;	// Applying error factor for odom calcs
			guardian_hw_interface->SetConversionFactor(MOTOR_GEARBOX_TO_TRACK_FACTOR);
		}
		else{
			distance_between_wheels_ *= ERROR_D_1;	// Applying error factor for odom calcs
		}*/
  		
  		distance_between_wheels_ *= error_factor_;
  		
		// Sets the max. speed read from the param
		guardian_hw_interface->SetSpeedLimits(max_linear_speed_, DTOR(max_angular_speed_));
		guardian_hw_interface->SetMotorWheelParams(diameter_wheels_, distance_between_wheels_);
		guardian_hw_interface->EnablePID(false);
		guardian_hw_interface->ToggleMotorPower(true);
		guardian_hw_interface->SetSpeed(0.0, 0.0);
		//
		// Setups and starts the component
		 // Setup
		while(guardian_hw_interface->Setup() == -1){
			sleep(t);
			ROS_ERROR("Main: Error in guardian setup. Trying it every %d seconds", t);
		}
		if(guardian_hw_interface->Start()!= OK){
		    ROS_ERROR("main: Error in guardian Start");
		}
		/*while(guardian_hw_interface->GetState() != READY_STATE){			
			ROS_ERROR("Main: Waiting until READY_STATE (%s). Trying it every %d seconds", guardian_hw_interface->GetStateString(), 2);
			sleep(2);
		} */
	}
	
	for(int i = 0; i < GUARDIAN_JOINTS; i++){
		robot_joints_.name.push_back("j");
		robot_joints_.position.push_back(0.0);
		robot_joints_.velocity.push_back(0.0);
		robot_joints_.effort.push_back(0.0);	
   }

	// Names of the joints
	robot_joints_.name[0] = back_left_wheel_joint_name_;
	robot_joints_.name[1] = front_left_wheel_joint_name_;
	robot_joints_.name[2] = back_right_wheel_joint_name_;
	robot_joints_.name[3] = front_right_wheel_joint_name_;
	

	//ROS_ERROR("Main: Controller on %s", guardian_hw_interface->GetStateString());
	// Define subscribers to obtain information through the sensors and joysticks
  	ros::Subscriber cmd_sub_ = pn.subscribe<geometry_msgs::Twist>("/guardian/cmd_vel", 1, cmdCallback);
	
	//ros::Subscriber io_sub_ = n.subscribe(modbus_io_topic_, 1, ioCallback);

	// Define publishers
	ros::Publisher odom_pub = pn.advertise<nav_msgs::Odometry>("/guardian/odom", 30);
	ros::Publisher state_pub = pn.advertise<guardian_node::guardian_state>("state", 30);
	tf::TransformBroadcaster odom_broadcaster;
	
	// Publish joint states for wheel motion visualization
	joint_state_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 10);

	//
	// Defining services
	ros::ServiceServer set_odometry_srv_ = pn.advertiseService("set_odometry",  set_odometry);
	//
	// Topics freq control 
	// For /guardian/cmd_vel
	double min_freq = GUARDIAN_MIN_COMMAND_REC_FREQ; // If you update these values, the
  	double max_freq = GUARDIAN_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
	sus_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/guardian/cmd_vel", updater_controller,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
	sus_command_freq->addTask(&command_freq); // Adding an additional task to the control
	//
	// For IO topic 
	double min_io_freq = 10.0; // If you update these values, the
  	double max_io_freq = 50.0; // HeaderlessTopicDiagnostic will use the new values.
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(desired_freq_);  // 50.0 
	
	//ROS_INFO("Main: Controller on %s", guardian_hw_interface->GetStateString());	

	//while( attemps < maxAttemps ) {

	while( ros::ok() ){ // n.ok() && (guardian->GetCurrentState() == 1)

		current_time = ros::Time::now();	
		
		// Obtain the odometry 
		robot_pose = guardian_hw_interface->GetPose();

		//ROS_INFO("Guardian_node: ChangeRefFrame");
		//ROS_WARN("*********************************\n");
		//ROS_INFO("Guardian_node: Main 2");
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

			if (invert_odom_tf_){ //Invert odom transform
				tf::Vector3 v(robot_pose.px, robot_pose.py, 0);
				tf::Quaternion q;
				tf::quaternionMsgToTF(odom_quat, q);
				tf::Transform tf_temp(q,v);
				tf_temp = tf_temp.inverse();

				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "base_footprint";
				odom_trans.child_frame_id = "odom_inverted";

				odom_trans.transform.translation.x = tf_temp.getOrigin().getX();
				odom_trans.transform.translation.y = tf_temp.getOrigin().getY();
				odom_trans.transform.translation.z = 0.0;

				geometry_msgs::Quaternion odom_quat2;
				tf::quaternionTFToMsg(tf_temp.getRotation(),odom_quat2);
				odom_trans.transform.rotation = odom_quat2;
			}
			else{

				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_footprint";

				odom_trans.transform.translation.x = robot_pose.px;
				odom_trans.transform.translation.y = robot_pose.py;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
			}

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
                                                       (0)   (0)   (0)  (0)  (0)  (1) ;

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
                                                       (0)   (0)   (0)  (0)  (0)  (1) ;
		//
		//publish the messages
		odom_pub.publish(odom);			// Odometry
		CheckRobotState();
		state_pub.publish(robot_state);	// State
		last_time = current_time;
		
		// Joint states
		robot_joints_.header.stamp = ros::Time::now();
		// Gets the current robot rpms
		guardian_hw_interface->GetRPM(&left_rpm, &right_rpm);
		// 1 rev -> 3.1416 rads
        // 1 min -> 60 segs
		// desired_freq_
		inc_left_wheel = ((left_rpm * 3.1416)/60)/desired_freq_;
		inc_right_wheel = ((right_rpm * 3.1416)/60)/desired_freq_;
		robot_joints_.position[0] = robot_joints_.position[0] + inc_left_wheel;
		robot_joints_.position[1] = robot_joints_.position[1] + inc_left_wheel;
		robot_joints_.position[2] = robot_joints_.position[2] + inc_right_wheel;
		robot_joints_.position[3] = robot_joints_.position[3] + inc_right_wheel;

		joint_state_pub_.publish( robot_joints_ );

		// UPDATING DIAGNOSTICS
		updater_controller.update();
			
		ros::spinOnce();
		r.sleep();
	}
		
	ROS_ERROR("guardian_node::main: ending program");
	
	guardian_hw_interface->SetSpeed(0, 0);
	guardian_hw_interface->ToggleMotorPower(false);
	guardian_hw_interface->Stop();

	delete guardian_hw_interface;
}


