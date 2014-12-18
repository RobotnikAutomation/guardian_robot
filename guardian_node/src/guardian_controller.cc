/** \file guardian_controller.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.1
 * \date    2012
 *
 * \brief guardian_controller class driver
 * Component to manage guardian_controller servo driver
 * Controller for Roboteq generation NTXGEN
 * (C) 2012 Robotnik Automation, SLL
*/
#include <ros/ros.h>
#include <guardian_node/guardian_controller.h>
#include <guardian_node/Constants.h>
#include <guardian_node/ErrorCodes.h>
#include <string.h>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <stdio.h>

using namespace std;

int max_left = 0; //apagar???
int max_right = 0;
/*!	\fn guardian_controller::guardian_controller(const char *device, double hz)
 * 	\brief Public parmetrized constructor
*/
guardian_controller::guardian_controller(const char *device, double hz): Component(hz) {

    // Initialization for odometry mutex
    pthread_mutex_init(& mutex_odometry, NULL);
    // Initialization of encoder mutex
    pthread_mutex_init(& mutex_encoders, NULL);
    // Component name
    sComponentName.assign("guardian_controller");
    // Create roboteq port
    roboteq = new RoboteqDevice();  //Creates roboteq device
    sDevice.assign(device);

	this->control_mode = -1; //CONTROL_RC;		    // By default control will be Radio Control
    this->encoders_mode = GUARDIAN_CONTROLLER_ABSOLUTE_ENCODERS;
    this->err_counter = 0;                          // Debug counts number of communication errors
    this->motor_control_mode = -1;                  // Unknown until reading

    this->iErrorType = 0;
    // Default PID constants values
    this->bPID = false;                         // By default the PID controller is not applied
    this->pidMotor.KiL = GUARDIAN_CONTROLLER_DEFAULT_I_L;                                 // I constant for linear velocity
    this->pidMotor.KiA = GUARDIAN_CONTROLLER_DEFAULT_I_A;                                 // I constant for angular velocity
    this->pidMotor.KpL_divisor = GUARDIAN_CONTROLLER_DEFAULT_P_L_DIVISOR;
    this->pidMotor.KpA_divisor = GUARDIAN_CONTROLLER_DEFAULT_P_A_DIVISOR;
    this->pidMotor.ErrSatIL_divisor = GUARDIAN_CONTROLLER_DEFAULT_ERR_SAT_L_DIVISOR;
    this->pidMotor.ErrSatIA_divisor = GUARDIAN_CONTROLLER_DEFAULT_ERR_SAT_A_DIVISOR;
    // Sets the maximum speed by default
    this->SetSpeedLimits(GUARDIAN_CONTROLLER_MAX_XSPEED, GUARDIAN_CONTROLLER_MAX_YAWSPEED);
    
	// Default values
    this->fDistanceBetweenWheels = MOTOR_D_WHEELS_M * ERROR_D_1;	// Wheel configuration by default
    this->fDiameterWheel = MOTOR_DIAMETER_WHEEL;					// 
	this->dCountsPerSec_To_RevPerMin = 60.0/MOTOR_COUNTS_PER_REV;	// Converts counts/sec into Rev/Min 
	this->dConversionFactor = 1.0;// By default, when robot has mounted the wheels

	this->robot_data.v_ref_mps = 0.0;
	this->robot_data.w_ref_rads = 0.0;
	this->robot_data.current_consumption[0] = this->robot_data.current_consumption[1] = 0;
	this->robot_data.temperature[0] = this->robot_data.temperature[1] = 0;
    // Configures PID values
    ConfigureConstants();
	// Configures constants by default for encoders
  	SetEncoderConfig(ROBOTEQ_DEFAULT_ENCODER_CONF, ROBOTEQ_DEFAULT_ENCODER_DIR, ROBOTEQ_DEFAULT_ANGULAR_DIR);
    
	ResetOdometry();
	this->robot_data.battery = 0.0;              // % of battery
    this->robot_data.voltage = 0.0;
    this->robot_data.voltage_internal  = 0.0;     // value used
    this->robot_data.digital_output = false;        // 1 digital output (real value)
    this->robot_data.digital_output_setvalue = false; // digital output (set value)
    this->robot_data.encoder_left = 0;           // left encoder pulses
    this->robot_data.encoder_right = 0;          // right encoder pulses
    this->robot_data.last_encoder_left = 0;      // left encoder pulses
    this->robot_data.last_encoder_right = 0;     // right encoder pulses
	this->robot_data.v_ref_mps = 0.0;	        // current reference linear speed [mps]
	this->robot_data.w_ref_rads = 0.0; 	        // current reference angular speed [rads]
    this->robot_data.rpm_left = 0.0;             // RPM motor left
    this->robot_data.rpm_right = 0;            // RPM motor right
    this->linearSpeedMps = 0.0;
    this->angularSpeedRads = 0.0;
    this->robot_data.channel_A_ref = this->robot_data.channel_B_ref = 0;
}

/*!	\fn guardian_controller::~guardian_controller()
 * 	\brief Public destructor
*/
guardian_controller::~guardian_controller(){

    // Delete roboteq port
    if (roboteq!=NULL)
        delete roboteq;

    // Destroy odometry mutex
	pthread_mutex_destroy(& mutex_odometry );

    // Destroy encoders mutex
	pthread_mutex_destroy(& mutex_encoders );

}

/*!	\fn ReturnValue guardian_controller::Open()
 * 	\brief Open device
 * 	\returns ERROR
 * 	\returns OK
*/
ReturnValue guardian_controller::Open(){

	// Setup and start guardian_controller device
	if(this->roboteq->Connect(sDevice) != RQ_SUCCESS) {
		ROS_ERROR("guardian_controller::Open: Error opening roboteq port");
		SwitchToState(FAILURE_STATE);
		this->iErrorType = GUARDIAN_CONTROLLER_ERROR_OPENING;
		return ERROR;
    }

    ROS_DEBUG("guardian_controller::Open: device opened successfully");

	return OK;
}

/*! \fn ReturnValue guardian_controller::Configure()
 * Configures devices and performance of the component
 * \return OK
 * \return ERROR if the configuration process fails
*/
ReturnValue guardian_controller::Configure(){
    int status = 0;
    if( (status = roboteq->SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
		ROS_ERROR("guardian_controller::Configure: failed --> %d", status);
	else
		ROS_DEBUG("guardian_controller::Configure succeeded");

    //Wait 10 ms before sending another command to device
	sleepms(10);

	return OK;
}

/*!	\fn ReturnValue guardian_controller::Close()
 * 	\brief Closes roboteq port
 * 	\returns ERROR
 * 	\returns OK
*/
ReturnValue guardian_controller::Close(){

    if (roboteq!=NULL) {
        roboteq->Disconnect();
		ROS_ERROR("guardian_controller::Close: roboteq disconnected");
    }

	return OK;
}

/*!	\fn void guardian_controller::ReadBatteryVoltage()
	* Reads the voltage
	* Sends the query to the controller.
*/
void guardian_controller::ReadBatteryVoltage(){
    int volts = 0;
    int status = 0;

    status = roboteq->GetValue(_VOLTS, 2, volts);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadBatteryVoltage: error = %d", status);
    }else{
        robot_data.voltage = (float)volts/10.0;
        robot_data.battery = CalculateBattery(robot_data.voltage);
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);
}

/*!	\fn void void guardian_controller::ReadTemperature()
 * 	\brief Read the internal temperature.
 *  Send the query to the controller.
 *  Command = ?m o ?M
*/
void guardian_controller::ReadTemperature(){
    int temp = 0;
    int status = 0;

    status = roboteq->GetValue(_TEMP, temp);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadTemperature: error = %d", status);
    }else{
        //ROS_DEBUG("guardian_controller::ReadTemperature: Temp = %d ", temp);
        //robot_data.temperature[0]=ValToHSTemp(HexToDec(&buf[2],2));
        //robot_data.temperature[1]=ValToHSTemp(HexToDec(&buf[4],2));
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);
}

/*!	\fn void void guardian_controller::ReadControlMode()
 * 	\brief Read the control mode
 *  Send the query to the controller.
 *  Command = ^01
*/
void guardian_controller::ReadMotorControlMode(){
    int mode = 0;
    int status = 0;

    status = roboteq->GetConfig(_MMOD, mode);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadMotorControlMode: error = %d ", status);
    }else{
        ROS_DEBUG("guardian_controller::ReadMotorControlMode: Mode =  %d\n", mode);
        motor_control_mode = mode;
        //robot_data.temperature[0]=ValToHSTemp(HexToDec(&buf[2],2));
        //robot_data.temperature[1]=ValToHSTemp(HexToDec(&buf[4],2));
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);
}

/*!	\fn void void guardian_controller::ReadInputControlMode()
 * 	\brief Read the current input control mode
 *  Send the query to the controller and read the response
 *  Command = ^00
*/
void guardian_controller::ReadInputControlMode(){
    /* int status = 0;

    status = roboteq->GetValue(_MMOD, &mode);
    if( (status != RQ_SUCCESS){
        cout << "guardian_controller::ReadMotorControlMode: error =  " << status << endl;
    }else{
        cout << "guardian_controller::ReadMotorControlMode: Temp =  " << mode << endl;
        motor_control_mode = mode;
        //robot_data.temperature[0]=ValToHSTemp(HexToDec(&buf[2],2));
        //robot_data.temperature[1]=ValToHSTemp(HexToDec(&buf[4],2));
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);*/

}

/*!	\fn void guardian_controller::WriteMotorControlMode(MotorControlMode mcm)
 * 	\brief Set the control mode
 *  Send the query to the controller.
 *  Command = ^01 C5 : mixed mode - closed loop
              ^01 01 : mixed mode - open loop
              ^01 00 : separate mode - open loop
*/
void guardian_controller::WriteMotorControlMode(MotorControlMode mcm){
    int mode = mcm;
    int status = 0;

    status = roboteq->SetConfig(_MMOD, mode);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::WriteMotorControlMode: error = %d", status);
    }else{
        ROS_DEBUG("guardian_controller::WriteMotorControlMode: Set mode %d", mode);
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);
}

/*!	\fn int guardian_controller::ReadAnalogInput(int number)
 * 	\brief Read analog inputs [FROM 1 TO GUARDIAN_CONTROLLER_ANALOG_INPUTS]
 * 	Send the query to the controller.
 *  \return 0 if OK
 *  \return status otherwise
*/
int guardian_controller::ReadAnalogInput(int number){
    int input = 0;
    int status = 0;

    if( (number <= 0) || (number > GUARDIAN_CONTROLLER_ANALOG_INPUTS) ){
        ROS_ERROR("guardian_controller::ReadAnalogInput: input %d out of range", number);
        return -1;
    }
    status = roboteq->GetValue(_ANAIN, number, input);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadAnalogInput: error = %d", status);
    }else{
        ROS_DEBUG("guardian_controller::ReadAnalogInput: Input[%d] = %d", number, input);
        robot_data.analog_input[number-1] = (float)input/1000.0;    // Converts from mV to V
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);

	return status;

}

/*!	\fn int guardian_controller::ReadDigitalInput(int number)
 * 	\brief Read digital input
 * 	Send the query to the controller.
*/
int guardian_controller::ReadDigitalInput(int number){
    int input = 0;
    int status = 0;

    if( (number <= 0) || (number > GUARDIAN_CONTROLLER_DIGITAL_INPUTS) ){
        ROS_ERROR("guardian_controller::ReadDigitalInput: input %d out of range", number);
        return -1;
    }
    status = roboteq->GetValue(_ANAIN, number, input);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadDigitalInput: error = %d", status);
    }else{
        ROS_DEBUG("guardian_controller::ReadDigitalInput: Input[%d] = %d", number, input);
        robot_data.digital_input[number-1] = input;
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);

	return status;
}


/*!	\fn void guardian_controller::ResetController()
 * 	\brief Allows the controller to be reset in the same manner as if the reset button
 * 	was pressed.
*/
void guardian_controller::ResetController(){


}

/*!	\fn void guardian_controller::EnterRS232Mode()
 * 	\brief If the controller is configured in R/C or Analog mode, it will not be able to accept and recognize
 * 	RS232 commands immediately. It will enter the roboteq mode after it has received 10 continuous
 * 	“Enter” (Carriage Return) characters
*/
void guardian_controller::EnterRS232Mode(){

}

/*!	\fn int guardian_controller::WriteDigitalOutput(bool value)
 * 	\brief Set the digital output into the selected value.
 *		 Initial configuration: output 1..4 managed by usbdux device
 *								output 5 managed by guardian_controller board
 * 	\param bit as int, output number from 1 to DIGITAL_OUTPUTS
 * 	\param value as int, output value
 * 	\return ERROR if the output can't be set
 * 	\return OK
*/
int guardian_controller::WriteDigitalOutput(bool value){

    return OK;
}


/*!	\fn void guardian_controller::ReadEncoders()
	* Read encoders value
*/
int guardian_controller::ReadEncoders(){
    int status = 0;
    int enc=0;
    bool bEncL = false, bEncR = false;

    // ENCODER LEFT
	//ROS_INFO("guardian_controller::ReadEncoders: LEFT");
    status = roboteq->GetValue(_ABCNTR, iChannelEncLeft, enc);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadEncoders: error = %d", status);
    }else{
        //ROS_INFO("guardian_controller::ReadEncoders: Encoder Left = %d", enc);
        this->robot_data.encoder_left = this->iEncoderDir * enc;
        bEncL = true;
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);

	// ENCODER RIGHT
	//ROS_INFO("guardian_controller::ReadEncoders: RIGHT");
    status = roboteq->GetValue(_ABCNTR, iChannelEncRight, enc);
    if(status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::ReadEncoders: error = %d", status);
    }else{
		//ROS_INFO("guardian_controller::ReadEncoders: Encoder Right = %d", enc);
        this->robot_data.encoder_right = this->iEncoderDir * enc;
        bEncR = true;
    }
    //Wait 10 ms before sending another command to device
	sleepms(10);

    if (bEncL && bEncR)
		return OK;
	else
		return ERROR;
}

/*! \fn int guardian_controller::ResetEncoders()
 *  \brief Resets the encoders counter with response confirmation
 *	\return OK
 *	\return ERROR
*/
int guardian_controller::ResetEncoders(){


	return OK;
}


/*! \fn void guardian_controller::WriteMotorSpeed( int  channel_a, int channel_b)
 * 	\brief Sets motor speed input variables are different variables according to guardian_controller cfg.
 *  \param channel_a  as int, the usage depends on the mode of operation
 *  \param channel_b  as int, the usage depends on the mode of operation
*/
void guardian_controller::WriteMotorSpeed(int channel_a, int channel_b){
    int status = 0;
//ROS_INFO("Setting motors: Desired  A= %d, B = %d", channel_a, channel_b);
    //Control max speed
    if((channel_a < 0) && (channel_a < -GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED) )
        channel_a = -GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED;
    else if( (channel_a > 0) && (channel_a > GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED) )
        channel_a = GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED;

    if((channel_b < 0) && (channel_b < -GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED) )
        channel_b = -GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED;
    else if( (channel_b > 0) && (channel_b > GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED) )
        channel_b = GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED;

    // CHANNEL A
	//ROS_INFO("guardian_controller::WriteMotorSpeed: setting channel A ( %d)", channel_a);
	status = roboteq->SetCommand(_GO, 1, channel_a);
    if( status != RQ_SUCCESS){
        ROS_ERROR("guardian_controller::WriteMotorSpeed: Error setting channel A (error = %d)", status);
    }else{

        sleepms(10);
		//ROS_INFO("guardian_controller::WriteMotorSpeed: setting channel B ( %d)", channel_b);
        status = roboteq->SetCommand(_GO, 2, channel_b);
        if( status != RQ_SUCCESS){
            ROS_ERROR("guardian_controller::WriteMotorSpeed: Error setting channel B (error = %d)", status);
        }
		
    }
    //Wait 10 ms before sending another command to device
    sleepms(10);

    // Saves the last references sent to the channels
    robot_data.channel_A_ref = channel_a;
    robot_data.channel_B_ref = channel_b;

}

/*!	\fn double guardian_controller::CalculateDeltaDistance()
	* Calculates distance travelled
	* @return sample period in seconds
*/
double guardian_controller::CalculateDeltaDistance(double *delta_left, double *delta_right){
	static bool first = true;
		static struct timespec now, last;
		int inc_left_counts = 0, inc_right_counts = 0;
		double secs= 1.0 / threadData.dRealHz;
		int diff= 0;
		double rev_per_min_left = 0.0, rev_per_min_right=0.0;
		static bool bfirst = true;


		// ABSOLUTE ENCODERS
		if(this->encoders_mode == GUARDIAN_CONTROLLER_ABSOLUTE_ENCODERS){
	        //pthread_mutex_lock(&mutex_encoders);
			if(bfirst){
				inc_left_counts = 0;
			    	inc_right_counts = 0;
			    	robot_data.last_encoder_left = robot_data.encoder_left;
			    	robot_data.last_encoder_right = robot_data.encoder_right;
				bfirst = false;
			}else{
				inc_left_counts = robot_data.encoder_left - robot_data.last_encoder_left;
			    inc_right_counts = robot_data.encoder_right - robot_data.last_encoder_right;
			    robot_data.last_encoder_left = robot_data.encoder_left;
			    robot_data.last_encoder_right = robot_data.encoder_right;
			}
	        //pthread_mutex_unlock(&mutex_encoders);
	        }
	    else {
	    // RELATIVE ENCODERS
	        //pthread_mutex_lock(&mutex_encoders);

	            inc_left_counts = robot_data.encoder_left;
	            inc_right_counts = robot_data.encoder_right;

				if(abs(inc_left_counts) > GUARDIAN_CONTROLLER_MAX_ENC_INC) {	// Check wrong data
	                ROS_ERROR("guardian_controller::CalculateRPM:Relative: error reading left encoder: inc = %d", inc_left_counts);
					inc_left_counts = robot_data.last_encoder_left;
				}else{
					robot_data.last_encoder_left = robot_data.encoder_left;
	            }

				if(abs(inc_right_counts) > GUARDIAN_CONTROLLER_MAX_ENC_INC){	//En caso de que llegase un dato anómalo
					ROS_ERROR("guardian_controller::CalculateRPM:Relative: error reading right encoder: inc = %d", inc_right_counts);
					inc_right_counts = robot_data.last_encoder_right;
				}else{
					robot_data.last_encoder_right = robot_data.encoder_right;
				}

				robot_data.encoder_left = 0;
				robot_data.encoder_right = 0;

	        //pthread_mutex_unlock(&mutex_encoders);
	    }

		//	Calcs rev per min for each motor
	/*
		*rpm_left= rev_per_min_left = (inc_left_counts/secs)*dCountsPerSec_To_RevPerMin;
		*rpm_right = rev_per_min_right = (inc_right_counts/secs)*dCountsPerSec_To_RevPerMin;
	*/
		*delta_left= (inc_left_counts)* DISTANCE_PER_COUNT;
		*delta_right= (inc_right_counts)* DISTANCE_PER_COUNT;

		//sprintf(aux, "guardian_controller::CalculateRPM: Rpm left = %lf(inc encoder = %d)\t Rpm Right = %lf(inc encoder = %d)\t time: diff= %lf secs\n", rev_per_min_left, inc_left_counts,rev_per_min_right,inc_right_counts,secs);

		return secs;
	}


/*!	\fn double guardian_controller::CalculateRPM()
	* Calculates RPM
	* @return sample period in seconds
*/
double guardian_controller::CalculateRPM(double *rpm_left, double *rpm_right){
	static bool first = true;
	static struct timespec now, last;
	int inc_left_counts = 0, inc_right_counts = 0;
	double secs= 1.0 / threadData.dRealHz;
	int diff= 0;
	double rev_per_min_left = 0.0, rev_per_min_right=0.0;
	static bool bfirst = true;


	// ABSOLUTE ENCODERS
	if(this->encoders_mode == GUARDIAN_CONTROLLER_ABSOLUTE_ENCODERS){
        //pthread_mutex_lock(&mutex_encoders);
		if(bfirst){
			inc_left_counts = 0;
		    	inc_right_counts = 0;
		    	robot_data.last_encoder_left = robot_data.encoder_left;
		    	robot_data.last_encoder_right = robot_data.encoder_right;
			bfirst = false;
		}else{
			inc_left_counts = robot_data.encoder_left - robot_data.last_encoder_left;
		    inc_right_counts = robot_data.encoder_right - robot_data.last_encoder_right;
		    robot_data.last_encoder_left = robot_data.encoder_left;
		    robot_data.last_encoder_right = robot_data.encoder_right;
		}
        //pthread_mutex_unlock(&mutex_encoders);
        }
    else {
    // RELATIVE ENCODERS
        //pthread_mutex_lock(&mutex_encoders);

            inc_left_counts = robot_data.encoder_left;
            inc_right_counts = robot_data.encoder_right;

			if(abs(inc_left_counts) > GUARDIAN_CONTROLLER_MAX_ENC_INC) {	// Check wrong data
                ROS_ERROR("guardian_controller::CalculateRPM:Relative: error reading left encoder: inc = %d", inc_left_counts);
				inc_left_counts = robot_data.last_encoder_left;
			}else{
				robot_data.last_encoder_left = robot_data.encoder_left;
            }

			if(abs(inc_right_counts) > GUARDIAN_CONTROLLER_MAX_ENC_INC){	//En caso de que llegase un dato anómalo
				ROS_ERROR("guardian_controller::CalculateRPM:Relative: error reading right encoder: inc = %d", inc_right_counts);
				inc_right_counts = robot_data.last_encoder_right;
			}else{
				robot_data.last_encoder_right = robot_data.encoder_right;
			}

			robot_data.encoder_left = 0;
			robot_data.encoder_right = 0;

        //pthread_mutex_unlock(&mutex_encoders);
    }
	
	//	Calcs rev per min for each motor

	*rpm_left= rev_per_min_left = (inc_left_counts/secs)*dCountsPerSec_To_RevPerMin;
	*rpm_right = rev_per_min_right = (inc_right_counts/secs)*dCountsPerSec_To_RevPerMin;

	//sprintf(aux, "guardian_controller::CalculateRPM: Rpm left = %lf(inc encoder = %d)\t Rpm Right = %lf(inc encoder = %d)\t time: diff= %lf secs\n", rev_per_min_left, inc_left_counts,rev_per_min_right,inc_right_counts,secs);
	
	return secs;
}


/*!	\fn void guardian_controller::UpdateSimpleOdometry()
	* Updates robot's odometry - only for tracks
*/
void guardian_controller::UpdateSimpleOdometry(){
	/*double fVelocityLeftRpm=0.0;
	double fVelocityRightRpm=0.0;*/
	double fVelocityLeft=0.0;
	double fVelocityRight=0.0;
	double v_left_mps = 0.0, v_right_mps = 0.0;
	double fSamplePeriod = 0.0; // Default sample period

	//fSamplePeriod= CalculateRPM(&fVelocityLeftRpm,&fVelocityRightRpm);
	fSamplePeriod= CalculateDeltaDistance(&fVelocityLeft,&fVelocityRight);

	// Convert velocities from rpm to m/s
	/* v_left_mps = fVelocityLeftRpm * dRpmToWheelRpm * dWheelRpmToMps;
	v_right_mps = fVelocityRightRpm * dRpmToWheelRpm * dWheelRpmToMps; */

	//already in m/s
	v_left_mps = fVelocityLeft/fSamplePeriod;
	v_right_mps = fVelocityRight/fSamplePeriod ;

	linearSpeedMps = (v_right_mps + v_left_mps) / 2.0;			   		    // m/s
	angularSpeedRads = (v_right_mps - v_left_mps) / fDistanceBetweenWheels;    //rad/s

    //Left and right track velocities
    robot_data.actSpeedLmps = v_left_mps;
    robot_data.actSpeedRmps = v_right_mps;
   // ROS_INFO("guardian_controller::CalcRPM OK. v_left_mps = %lf,  v_right_mps = %lf",v_left_mps,v_right_mps);

	//Velocity //this is converting velocity to the odom frame
	/*robot_pose.va = angularSpeedRads;
	robot_pose.vx = linearSpeedMps *  cos(robot_pose.pa);
	robot_pose.vy = linearSpeedMps *  sin(robot_pose.pa);
	//ROS_INFO("guardian_controller::CalcRPM OK. pa = %lf,  w = %lf, f = %lf", robot_pose.pa, angularSpeedRads, fSamplePeriod);

	//Position
	robot_pose.pa += angularSpeedRads * fSamplePeriod;
	radnorm(&robot_pose.pa);

	robot_pose.px += robot_pose.vx * fSamplePeriod;
	robot_pose.py += robot_pose.vy * fSamplePeriod;
	//ROS_INFO("Pos x = %lf, Pos y = %lf, heading = %lf",robot_pose.px,robot_pose.py,robot_pose.pa);

	robot_data.rpm_right = (float)fVelocityRightRpm;   // Motor RPM
	robot_data.rpm_left  = (float)fVelocityLeftRpm;*/

    //Velocity
    //we want velocity in frame base_footprint
    	robot_pose.va = angularSpeedRads;
    	robot_pose.vx = linearSpeedMps ; //*  cos(robot_pose.pa); //robot only moves in x coordinate
    	robot_pose.vy = 0; //linearSpeedMps *  sin(robot_pose.pa);
    	//ROS_INFO("guardian_controller::CalcRPM OK. pa = %lf,  w = %lf, f = %lf", robot_pose.pa, angularSpeedRads, fSamplePeriod);

    	//Position
    	robot_pose.pa += angularSpeedRads * fSamplePeriod;
    	radnorm(&robot_pose.pa);

    //but the pose we want in odom frame, so the multiplication (cos, sin) is done here
    	robot_pose.px += robot_pose.vx * cos(robot_pose.pa)* fSamplePeriod;
    	robot_pose.py += robot_pose.vx * sin(robot_pose.pa)* fSamplePeriod;
    	//ROS_INFO("Pos x = %lf, Pos y = %lf, heading = %lf",robot_pose.px,robot_pose.py,robot_pose.pa);

    	//robot_data.rpm_right = (float)fVelocityRightRpm;// Motor RPM
    	//robot_data.rpm_left  = (float)fVelocityLeftRpm;

    	robot_data.rpm_right = (float)fVelocityRight*dMpsToWheelRpm;  //now convert to rpm
    	robot_data.rpm_left  = (float)fVelocityLeft*dMpsToWheelRpm;

}

/*!	\fn void guardian_controller::UpdateOdometry()
	* Updates robot's odometry
*/
void guardian_controller::UpdateOdometry(){
	double fVelocityLeftRpm=0.0;
	double fVelocityRightRpm=0.0;
	/*double fVelocityLeft=0.0;
	double fVelocityRight=0.0;*/
	double v_left_mps = 0.0, v_right_mps = 0.0;
	double fSamplePeriod = 0.0; // Default sample period

	fSamplePeriod= CalculateRPM(&fVelocityLeftRpm,&fVelocityRightRpm);
	//fSamplePeriod= CalculateRPM(&fVelocityLeft,&fVelocityRight);

	// Convert velocities from rpm to m/s
	v_left_mps = fVelocityLeftRpm * dRpmToWheelRpm * dWheelRpmToMps;
	v_right_mps = fVelocityRightRpm * dRpmToWheelRpm * dWheelRpmToMps;
	
	//already in m/s
	/*v_left_mps = fVelocityLeft/fSamplePeriod;
	v_right_mps = fVelocityRight/fSamplePeriod ;*/

	linearSpeedMps = (v_right_mps + v_left_mps) / 2.0;			   		    // m/s
	angularSpeedRads = (v_right_mps - v_left_mps) / fDistanceBetweenWheels;    //rad/s

    //Left and right track velocities
    robot_data.actSpeedLmps = v_left_mps;
    robot_data.actSpeedRmps = v_right_mps;
   // ROS_INFO("guardian_controller::CalcRPM OK. v_left_mps = %lf,  v_right_mps = %lf",v_left_mps,v_right_mps);
	
	//Velocity //this is converting velocity to the odom frame
	/*robot_pose.va = angularSpeedRads;
	robot_pose.vx = linearSpeedMps *  cos(robot_pose.pa);
	robot_pose.vy = linearSpeedMps *  sin(robot_pose.pa);
	//ROS_INFO("guardian_controller::CalcRPM OK. pa = %lf,  w = %lf, f = %lf", robot_pose.pa, angularSpeedRads, fSamplePeriod);

	//Position
	robot_pose.pa += angularSpeedRads * fSamplePeriod;
	radnorm(&robot_pose.pa);

	robot_pose.px += robot_pose.vx * fSamplePeriod;
	robot_pose.py += robot_pose.vy * fSamplePeriod;
	//ROS_INFO("Pos x = %lf, Pos y = %lf, heading = %lf",robot_pose.px,robot_pose.py,robot_pose.pa);

	robot_data.rpm_right = (float)fVelocityRightRpm;   // Motor RPM
	robot_data.rpm_left  = (float)fVelocityLeftRpm;*/

    //Velocity 
    //we want velocity in frame base_footprint 
    	robot_pose.va = angularSpeedRads;
    	robot_pose.vx = linearSpeedMps ; //*  cos(robot_pose.pa); //robot only moves in x coordinate
    	robot_pose.vy = 0; //linearSpeedMps *  sin(robot_pose.pa);
    	//ROS_INFO("guardian_controller::CalcRPM OK. pa = %lf,  w = %lf, f = %lf", robot_pose.pa, angularSpeedRads, fSamplePeriod);

    	//Position
    	robot_pose.pa += angularSpeedRads * fSamplePeriod;
    	radnorm(&robot_pose.pa);

    //but the pose we want in odom frame, so the multiplication (cos, sin) is done here
    	robot_pose.px += robot_pose.vx * cos(robot_pose.pa)* fSamplePeriod;
    	robot_pose.py += robot_pose.vx * sin(robot_pose.pa)* fSamplePeriod;
    	//ROS_INFO("Pos x = %lf, Pos y = %lf, heading = %lf",robot_pose.px,robot_pose.py,robot_pose.pa);

    	robot_data.rpm_right = (float)fVelocityRightRpm;// Motor RPM
    	robot_data.rpm_left  = (float)fVelocityLeftRpm;

    	//robot_data.rpm_right = (float)fVelocityRight*dMpsToWheelRpm;  //now convert to rpm
    	//robot_data.rpm_left  = (float)fVelocityLeft*dMpsToWheelRpm;

}

/*!\fn void guardian_controller::SetVWRef()
 * guardian_controller state space control of V:Linear speed and W:Angular speed
 * guardian_controller configured AB mixed, closed loop
*/
void guardian_controller::SetVWRef()
{
    int channelA = 0, channelB = 0;
    int uv = 0, uw = 0;
   //
   // IF we want software PID control ...
    if(bPID){
        static double err_lin_i = 0.0, err_ang_i = 0.0;
        double err_lin = robot_data.v_ref_mps  - linearSpeedMps;
        double err_ang = robot_data.w_ref_rads - angularSpeedRads;

        err_lin_i += err_lin;
        err_ang_i += err_ang;

        // Checks the
        if (err_lin_i > pidMotor.ErrSatIL) err_lin_i = pidMotor.ErrSatIL;
        else if (err_lin_i < -pidMotor.ErrSatIL) err_lin_i = -pidMotor.ErrSatIL;

        if (err_ang_i > pidMotor.ErrSatIA) err_ang_i = pidMotor.ErrSatIA;
        else if (err_ang_i < -pidMotor.ErrSatIA) err_ang_i = -pidMotor.ErrSatIA;

        uv = (int) (pidMotor.KpL * err_lin + pidMotor.KiL * err_lin_i);
        uw =  (int) (pidMotor.KpA  * err_ang + pidMotor.KiA * err_ang_i);
    }

    //ROS_INFO("SetVWRef: l= %.3lf, w = %.3lf", robot_data.v_ref_mps, robot_data.w_ref_rads);
	channelA = (int) iEncoderDir * (robot_data.v_ref_mps * dMpsToRef + uv);
    channelB = (int) iAngularSpeedDir * (robot_data.w_ref_rads * (dMpsToRef * fDistanceBetweenWheels) * 0.5 + uw );

	//channelA = (int) (GUARDIAN_CONTROLLER_ENCODER_DIR * (robot_data.v_ref_mps * dMpsToRef + uv));
    //channelB = (int) (-GUARDIAN_CONTROLLER_ENCODER_DIR * (robot_data.w_ref_rads * (dMpsToRef / fDistanceBetweenWheels) + uw ));

	// ROS_INFO("Channel A (%d) = %f * (%lf * %lf + %d)", channelA, GUARDIAN_CONTROLLER_ENCODER_DIR, robot_data.v_ref_mps, dMpsToRef, uv);
    // ROS_INFO("Channel B (%d) = %f * (%lf * (%lf / %lf) + %d)", channelB, GUARDIAN_CONTROLLER_ENCODER_DIR, robot_data.w_ref_rads, dMpsToRef,  fDistanceBetweenWheels, uw);
    //ROS_INFO("SetVWRef: l= %.3lf (%d), w = %.3lf (%d)", robot_data.v_ref_mps, channelA, robot_data.w_ref_rads, channelB);

	WriteMotorSpeed(channelA, channelB);

}

/*! \fn void guardian_controller::InitState()
   * Component State Machine InitState
   * Periodic actions at component hz
*/
void guardian_controller::InitState()
{
    // Enter roboteq communication mode
    //EnterRS232Mode();

    //WriteMotorControlMode(MCM_MIXED_CLOSED);

    // Reset Encoders (avoid initial odometry error due to wrong initialization
    if (ResetEncoders()!=OK) {
        this->iErrorType= GUARDIAN_CONTROLLER_ERROR_SERIALCOMM;
        SwitchToState(FAILURE_STATE);
        return;
    } else {
		WriteMotorSpeed(0, 0);
        this->iErrorType=GUARDIAN_CONTROLLER_ERROR_NONE;
        sleep(1);
        SwitchToState(READY_STATE);
	}

}

/*! \fn void guardian_controller::ReadyState()
   * Component State Machine ReadyState
   * Periodic actions at component hz
*/
void guardian_controller::ReadyState()
{
    static int token = 0;

    // Send cmd and process response
    if (ReadEncoders() == OK){
        UpdateOdometry();
	}else {
       this->iErrorType = GUARDIAN_CONTROLLER_ERROR_SERIALCOMM;
       SwitchToState(FAILURE_STATE);
    }

    if (robot_data.bMotorsEnabled) {
        // Control robot axes
        SetVWRef();
	}
    else
        // Set 0 references
        WriteMotorSpeed(0, 0);

    // No time critical functions called iteratively
    switch (token) {

        case 0:
            ReadBatteryVoltage(); // Send cmd and process response
        break;

        case 1:
            //ReadAnalogInputs();   // Read analog inputs
        break;

        case 2:
            ReadTemperature();    // Read temperatures
        break;

        case 3:
            //ReadDigitalInputs();  // Read digital inputs
        break;

        case 4: //WriteDigitalOutput( robot_data.digital_output_setvalue );
            ReadMotorControlMode();
        break;

        case 5:
            ReadInputControlMode();
            token=-1;
        break;

        default:token=-1;
                break;
        }

	token++;
	
}

/*!	\fn void guardian_controller::FailureState()
 * 	\brief Actions in Failure State
*/
void guardian_controller::FailureState(){
	int timer = 2500000; //useconds
	static int recovery_cycles = 0;

	recovery_cycles++;
	if(recovery_cycles >= 5*GUARDIAN_CONTROLLER_DEFAULT_HZ){ // Try to recover each second
		switch(iErrorType)	{

			ROS_DEBUG("guardian_controller::FailureState: Trying to recover..");

			case GUARDIAN_CONTROLLER_ERROR_OPENING: //Try to recover
				ROS_ERROR("guardian_controller::FailureState: Recovering from failure state (ERROR_OPENING)");
				this->Close();
				usleep(timer);
				if (this->Open()==OK)
                    SwitchToState(INIT_STATE);
				break;

			case GUARDIAN_CONTROLLER_ERROR_SERIALCOMM:
				ROS_ERROR("guardian_controller::FailureState: Recovering from failure state (ERROR_SERIALCOMM)");
				this->Close();
				usleep(timer);
				if (this->Open()==OK){                    
                    SwitchToState(INIT_STATE);
				}

				break;

/*
			case ERROR_TIMEOUT:
				log->AddError((char*)"guardian_controller::FailureState: Recovering from failure state (ERROR_TIMEOUT.)");
				printf("guardian_controller::FailureState: Recovering from failure state (ERROR_TIMEOUT.)\n");
				Close();
				usleep(timer);
				Open();
				break;
*/
		}
		recovery_cycles = 0;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//
// PUBLIC FUNCTIONS / DATA ACCESS FUNCTIONS
//
///////////////////////////////////////////////////////////////////////////////////////////////////

/*!	\fn void guardian_controller::ToggleMotorPower(bool val)
 * 	\brief Switches on/off the motor
*/
void guardian_controller::ToggleMotorPower(bool val)
{
    this->robot_data.bMotorsEnabled = val;
    ROS_INFO("guardian_controller::ToggleMotorPower: Motor %s",val?"enabled":"disabled");
	//SwitchToState(READY_STATE);
}


/*!	\fn void guardian_controller::GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa)
	* Returns robot velocity and pose
*/
void guardian_controller::GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa)
{
    //pthread_mutex_lock(&mutex_odometry);
        // Return values
        *vx = robot_pose.vx; *vy=robot_pose.vy; *va=robot_pose.va;
        *px=robot_pose.px; *py=robot_pose.py; *pa=robot_pose.pa;
    //pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn pose guardian_controller::GetPose()
	* Returns the Pose (pos & vel)
*/
pose guardian_controller::GetPose(){
    return robot_pose;
}

/*!	\fn void guardian_controller::ResetOdometry()
	* Resets driver odometry
*/
void guardian_controller::ResetOdometry(){

	// Reset player local copy of the robot odometry
	//pthread_mutex_lock(&mutex_odometry);
		robot_pose.px = 0;
		robot_pose.py = 0;
		robot_pose.pa = 0;
		robot_pose.vx = 0;
		robot_pose.vy = 0;
		robot_pose.va = 0;
	//pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void guardian_controller::ModifyOdometry(double px,  double py,  double pa)
	* Set new odometry value (pose)
*/
void guardian_controller::ModifyOdometry(  double px,  double py,  double pa )
{
	// Reset player local copy of the robot odometry
	//pthread_mutex_lock(&mutex_odometry);
		robot_pose.px = px;
		robot_pose.py = py;
		robot_pose.pa = pa;
	//pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void guardian_controller::SetSpeedLimits()
	* Set robot linear and angular speed limits
	* linear speed in m/s
	* angular speed in rad/s
*/
void guardian_controller::SetSpeedLimits(float lin_speed, float ang_speed){

    robot_data.max_linear_speed = fabs(lin_speed);  // m/s
    robot_data.max_angular_speed = fabs(ang_speed); // rad/s
	ROS_INFO("guardian_controller::SetSpeedLimits: Max speed = Linear %lf m/s, angular %lf rad/s", robot_data.max_linear_speed, robot_data.max_angular_speed);
}

/*!	\fn void guardian_controller::SetSpeed(double lin_speed, double ang_speed)
	* Set robot's left and right track references from linear and angular values
	* \param lin_speed as double, desired linear speed
	* \param ang_speed as double, desired angular speed
*/
void guardian_controller::SetSpeed(double lin_speed, double ang_speed){
	//ROS_INFO("SetSpeed: %lf, %lf", lin_speed, ang_speed);
	//pthread_mutex_lock(&mutex_odometry);
        // Check the limits of the desired velocity
        if(fabs(lin_speed) > robot_data.max_linear_speed){
            if(lin_speed < 0.0)
                robot_data.v_ref_mps = -robot_data.max_linear_speed;
            else
                robot_data.v_ref_mps = robot_data.max_linear_speed;
        }else
             robot_data.v_ref_mps = lin_speed;   // Sets desired linear speed

        if(fabs(ang_speed) > robot_data.max_angular_speed){
            if(ang_speed < 0.0)
                robot_data.w_ref_rads = -robot_data.max_angular_speed;
            else
                robot_data.w_ref_rads = robot_data.max_angular_speed;
        }else
            robot_data.w_ref_rads = ang_speed;  // Sets desired angular speed

	//pthread_mutex_unlock(&mutex_odometry);
	
}

/*!	\fn void guardian_controller::GetVoltage()
	* Return the robot voltage value measured internally
*/
float guardian_controller::GetVoltage()
{
    return robot_data.voltage;
}

/*!	\fn int guardian_controller::GetEncoder(char enc)
	* Return the last value measured of encoder 'L'eft or 'R'ight
*/
int guardian_controller::GetEncoder(char enc)
{
    int encoder_value=0;
    switch (enc) {
        case 'L':
        case 'l':
            //pthread_mutex_lock(&mutex_encoders);
                encoder_value = robot_data.encoder_left;
            //pthread_mutex_unlock(&mutex_encoders);
            break;
        case 'R':
        case 'r':
            //pthread_mutex_lock(&mutex_encoders);
                encoder_value = robot_data.encoder_right;
            //pthread_mutex_unlock(&mutex_encoders);
            break;
        }

    return encoder_value;
}

/*!	\fn int guardian_controller::GetTemperature(int index)
	* Return the heat sink temperature measurement
*/
int guardian_controller::GetTemp(int index)
{
    if ((index<0)||(index>1)) return 0;
    else return robot_data.temperature[index];
}

/*!	\fn float guardian_controller::GetTemperature(int index)
	* Return the selected analog input measurement
*/
float guardian_controller::GetAnalogInput(int index)
{
    if ((index<0)||(index>1)) return 0;
    else return robot_data.analog_input[index];
}

/*!	\fn float guardian_controller::SetDigitalOutput
	* Public function for setting /resetting digital output
*/
void guardian_controller::SetDigitalOutput( bool value )
{
   robot_data.digital_output_setvalue = value;
}

/*!	\fn float guardian_controller::GetDigitalOutput
	* Public function for reading digital output
*/
bool guardian_controller::GetDigitalOutput()
{
   return robot_data.digital_output;
}

/*!	\fn bool guardian_controller::GetDigitalInput
	* Public function for reading digital inputs 0..2
*/
bool guardian_controller::GetDigitalInput(int input)
{
   if ((input>=0) && (input<=2))
      return robot_data.digital_input[input];
   else
      return false;
}

/*!	\fn float guardian_controller::CalculateBattery(double voltage)
	* Calculates the battery level using the discharge curve
*/
float guardian_controller::CalculateBattery(double voltage){
    // 53 V -> 100%
    // 51 V   -> 20%
    // 50.2 V -> 10%
    // 49 V   -> 0.1%
    double vLevels[4][2]={{53, 100}, {51, 20}, {50.2, 10}, {49, 0.1}};
    int size = 4;
    double b = 0, m = 0, result = -1.0;

    if(voltage >= vLevels[0][0])
        return 100.0;

    if(voltage < vLevels[size - 1][0])
        return 0.0;

    for(int i = 0; i < size-2; i++){
        if((voltage < vLevels[i][0]) && ( voltage >=  vLevels[i+1][0])){
            m = (vLevels[i+1][1] - vLevels[i][1]) / (vLevels[i+1][0] - vLevels[i][0]);
            b = vLevels[i][1] - m * vLevels[i][0];
            result = m*voltage + b;

            return result;
        }
    }
    return result;

}

/*!	\fn double guardian_controller::GetBatteryPercent()
	* Gets the percent of the total battery
*/
double guardian_controller::GetBatteryPercent(){
    return robot_data.battery;
}

/*!	\fn int guardian_controller::GetInputControlMode()
	* Returns the current input control mode (RC, RS232,  ..)
*/
int guardian_controller::GetInputControlMode(){
    return control_mode;
}

/*!	\fn const char * guardian_controller::GetInputControlMode()
	*  Returns the current input control mode (RC, RS232,  ..)
*/
const char * guardian_controller::GetInputControlModeString(){
	switch(control_mode){
		case ICM_RC_MODE:
			return "RC";
		break;
		case ICM_RS232:
			return "RS232";
		break;
		case ICM_RS232_WATCHDOG:
			return "RS232+WATCHDOG";
		break;
		case ICM_ANALOG:
            return "ANALOG";
		break;
		default:
			return "UNKNOWN";
		break;
	}
}

/*!	\fn int guardian_controller::GetMotorControlMode()
	* Returns the current input control mode (MIXED, SEPARATED,  ..)
*/
int guardian_controller::GetMotorControlMode(){
    return motor_control_mode;
}

/*!	\fn const char * guardian_controller::GetMotorControlMode()
	*  Returns the current Motor control mode (MIXED, SEPARATED, ..)
*/
const char * guardian_controller::GetMotorControlModeString(){
	switch(motor_control_mode){
		case MCM_OPEN_SPEED:
			return "OPEN SPEED";
		break;
		case MCM_CLOSED_SPEED:
			return "CLOSED SPEED";
		break;
		case MCM_CLOSED_POSITION:
			return "CLOSED POSITION";
		break;
		default:
			return "UNKNOWN";
		break;
	}
}

/*!	\fn void guardian_controller::GetRPM(float *left_rpm, float *right_rpm)
	* Gets the rpm of each motor
*/
void guardian_controller::GetRPM(float *left_rpm, float *right_rpm){
    *left_rpm = robot_data.rpm_left;
    *right_rpm = robot_data.rpm_right;
}

/*!	\fn void guardian_controller::GetChannelReference(int *a, int *b)
	* Gets the channel's reference sent to the controller
*/
void guardian_controller::GetChannelReference(int *a, int *b){
    *a = robot_data.channel_A_ref;
    *b = robot_data.channel_B_ref;
}

/*!	\fn void guardian_controller::ConfigureConstants()
	* Configs the constants parameters
*/
void guardian_controller::ConfigureConstants(){
    // Constants to control the speed
	
	// Constants to control the speed
	dWheelRpmToMps = (fDiameterWheel * Pi) / 60.0;          // conversion value from wheels rpm to mps = PI*Diameter/60
	dRpmToRef = (double) (GUARDIAN_CONTROLLER_REF_FOR_MAX_RPM / MOTOR_MAX_RPM);     // conversion from MOTOR RPM to Reference        
	dRpmToWheelRpm = (double) (MOTOR_GEARBOX * dConversionFactor); 	// Conversion from MOTOR_RPM  to WHEEL RPM

	dWheelRpmToRpm = 1.0 / dRpmToWheelRpm;					// conversion from Wheel RPM to motor RPM
	dMpsToWheelRpm = 1.0 / dWheelRpmToMps;                    // conversion from MPS to wheels RPM
	dRefToRpm = 1.0 / dRpmToRef;                              // Conversion from Reference to MOTOR RPM

	dMpsToRef = dMpsToWheelRpm * dWheelRpmToRpm * dRpmToRef;        // Conversion from MPS to motor Reference
	dRefToMps = 1.0 / dMpsToRef;                                      // Conversion from Reference to MPS

    /*dWheelRpmToMps = (fDiameterWheel * Pi) / 60.0;          // conversion value from wheels rpm to mps = PI*Diameter/60
    dRpmToRef = GUARDIAN_CONTROLLER_REF_FOR_MAX_RPM / MOTOR_MAX_RPM;     // conversion from MOTOR RPM to Reference

    dMpsToWheelRpm = 1 / dWheelRpmToMps;                         // conversion from MPS to wheels RPM
    dWheelRpmToRpm = MOTOR_GEARBOX / dConversionFactor;          // conversion from Wheel RPM to motor RPM
    dRpmToWheelRpm = 1/ dWheelRpmToRpm;

    dRefToRpm = 1 / dRpmToRef;                               // Conversion from Reference to MOTOR RPM

    dMpsToRef = dMpsToWheelRpm * dWheelRpmToRpm * dRpmToRef;        // Conversion from MPS to motor Reference
    dRefToMps = 1 / dMpsToRef;                                      // Conversion from Reference to MPS*/

    // PI parameters for velocity control. D parameter is not used
    pidMotor.KpL = (GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_REF_SPEED / 2.0)* pidMotor.KpL_divisor;        // P constant for linear velocity
    pidMotor.KpA = (GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_REF_SPEED / (Pi / 2.0))*pidMotor.KpA_divisor;  // P constant for angular velocity
    pidMotor.ErrSatIL = (WHEELS_MAX_RPM*dRpmToWheelRpm) * pidMotor.ErrSatIL_divisor;                               // Max Linear Saturation Error: 10% of the max linear speed
    pidMotor.ErrSatIA = (2*(WHEELS_MAX_RPM*dRpmToWheelRpm) / fDistanceBetweenWheels) * pidMotor.ErrSatIA_divisor;  // Max Angular Saturation Error: 10% of the max angular speed
}

/*!	\fn void guardian_controller::GetRobotSpeedLimits(float *lin_speed, float *ang_speed)
	* Gets the maximum configured speeds
*/
void guardian_controller::GetRobotSpeedLimits(float *lin_speed, float *ang_speed){
    *lin_speed = robot_data.max_linear_speed;
    *ang_speed = robot_data.max_angular_speed;
}

/*!	\fn void guardian_controller::SetMotorWheelParams(float diameter, float distance){
	* Sets the wheels' diameter and the distance between them
*/
void guardian_controller::SetMotorWheelParams(float diameter, float distance){
    if(diameter != 0.0 && distance != 0.0){
        fDiameterWheel = fabs(diameter);
        fDistanceBetweenWheels = fabs(distance);
        // Reconfigure PID parameters for the new values
        ConfigureConstants();
    }
}

/*!	\fn void guardian_controller::SetP(float p_linear_divisor, float p_angular_divisor)
	* Sets the P adjustable parameters
*/
void guardian_controller::SetP(float p_linear_divisor, float p_angular_divisor){
    pidMotor.KpL_divisor = p_linear_divisor;
    pidMotor.KpA_divisor = p_angular_divisor;
    // Reconfigures PID
    ConfigureConstants();
}

/*!	\fn void guardian_controller::SetI(float i_linear_constant, float i_angular_constant, float i_saturation_linear_error_divisor, float i_saturation_angular_error_divisor)
	* Sets the I adjustable parameters
*/
void guardian_controller::SetI(float i_linear_constant, float i_angular_constant, float i_saturation_linear_error_divisor, float i_saturation_angular_error_divisor){
    pidMotor.KiL = i_linear_constant;
    pidMotor.KiA = i_angular_constant;
    pidMotor.ErrSatIL_divisor = i_saturation_linear_error_divisor;
    pidMotor.ErrSatIA_divisor = i_saturation_angular_error_divisor;
    // Reconfigures PID
    ConfigureConstants();
}

/*!	\fn void guardian_controller::EnablePID(bool value)
	* Enables / Disables the PID controller
*/
void guardian_controller::EnablePID(bool value){
    bPID = value;
}

/*!	\fn void guardian_controller::EnablePID(bool value)
	* Sets the PID parametes
*/
void guardian_controller::SetPID(pid_params new_pid){
    pidMotor = new_pid;
    // Reconfigures PID
    ConfigureConstants();
}

/*!	\fn pid_params guardian_controller::GetPID()
	* Gets current PID params
*/
pid_params guardian_controller::GetPID(){
    return pidMotor;
}

/*! \fn roboteq_data guardian_controller::GetData()
 * 	\brief Gets all the data of the guardian_controller
*/
roboteq_data guardian_controller::GetData(){
    return robot_data;
}

/*! \fn bool guardian_controller::IsMotorEnabled()
 * 	\brief Return true if the motor is enabled
*/
bool guardian_controller::IsMotorEnabled(){
    return robot_data.bMotorsEnabled;
}

/*! \fn bool guardian_controller::IsPIDEnable()
 * 	\brief Return true if the PID is enabled
*/
bool guardian_controller::IsPIDEnable(){
    return bPID;
}

/*! \fn void guardian_controller::SetConversionFactor(double val)
 * 	\brief Sets the conversion factor (attribute dConversionFactor)
*/
void guardian_controller::SetConversionFactor(double val){
	dConversionFactor = val;
	ConfigureConstants();
}

/*! \fn void guardian_controller::GetDesiredSpeed(float *linear, float *angular)
 * 	\brief Gets the desired linear and angular speed
*/
void guardian_controller::GetDesiredSpeed(float *linear, float *angular){
	*linear = (float) robot_data.v_ref_mps;
	*angular = (float) robot_data.w_ref_rads;
}


/*! \fn void guardian_controller::SetEncoderConfig(int config, int direction, int angular_dir)
 * 	\brief Sets the internal encoders' configuration
*/
void guardian_controller::SetEncoderConfig(int config, int direction, int angular_dir){
	this->iEncoderConfig = config;
	this->iEncoderDir = direction;
	this->iAngularSpeedDir = angular_dir;
	
   	// Configure commands according to driver encoders mode
    if(this->iEncoderConfig == 1){
        iChannelEncLeft = 1;
        iChannelEncRight = 2;
    }else{
        iChannelEncLeft = 2;
        iChannelEncRight = 1;
    }
}
