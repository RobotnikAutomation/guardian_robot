/** \file guardian_controller.h
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2012
 *
 * \brief class for guardian_controller servo driver
 * (C) 2012 Robotnik Automation, SLL
*/

#ifndef __GUARDIAN_CONTROLLER_H
	#define __GUARDIAN_CONTROLLER_H

#include "guardian_node/Component.h"
#include "guardian_node/RoboteqDevice.h"
#include <stdint.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */

// conection stuff
#define GUARDIAN_CONTROLLER_DEFAULT_PORT 		        "/dev/ttyS1"
#define GUARDIAN_CONTROLLER_DEFAULT_PARITY 		        "even"
#define GUARDIAN_CONTROLLER_DEFAULT_BAUDRATE 	        9600
#define GUARDIAN_CONTROLLER_DEFAULT_DATA_SIZE 	        7

// component frequency
#define GUARDIAN_CONTROLLER_DEFAULT_HZ                   10

// Default max speeds
#define GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_SPEED			1000            //
#define GUARDIAN_CONTROLLER_MOTOR_DEF_MAX_REF_SPEED		4000         //1000

// CONTROL MODE
#define GUARDIAN_CONTROLLER_CONTROL_RS232				1
#define GUARDIAN_CONTROLLER_CONTROL_RC					2

// INTERNAL ERROR CODES
#define GUARDIAN_CONTROLLER_ERROR_NONE				    0
#define GUARDIAN_CONTROLLER_ERROR_OPENING			    1
#define GUARDIAN_CONTROLLER_ERROR_SERIALCOMM		 	2
#define GUARDIAN_CONTROLLER_ERROR_TIMEOUT			    3

// ODOMETRY PARAMETERS
#define ERROR_D_1							1.80
#define ERROR_D_2							1.25
#define ODOMTYPE_WHEELS						"wheels"		
#define ODOMTYPE_TRACKS						"tracks"
#define MOTOR_COUNTS_PER_REV		     	2000            // =500 x 4 (in quadrature mode)
#define MOTOR_GEARBOX					    1.0/40.0        // gearbox 1:40
#define MOTOR_DIAMETER_WHEEL			    0.36		    // m (Diameter by default of the wheel)
#define MOTOR_DIAMETER_TRACK_WHEEL		    0.2		        // m (Diameter of the wheel used for "tracks" odometry)
#define MOTOR_GEARBOX_TO_TRACK_FACTOR		21.0/15.0		// Factor applied to convert motor gearbox spins to track spin
//#define MOTOR_RPM2MPS			            0.012790994     // conversion value from rpm to mps PI*Diameter/60          // 
#define MOTOR_D_TRACKS_M	                0.448          	// theorical distance between motor tracks 
#define MOTOR_D_WHEELS_M	                0.650           // theorical distance between motor wheels
#define MOTOR_MAX_RPM                       2900.0           // Motor specs
#define WHEELS_MAX_RPM                      (MOTOR_MAX_RPM * MOTOR_GEARBOX) // Max RPM of each wheel depending on the motor's specs
#define DISTANCE_PER_COUNT 					3.14*0.2/57110  //tracks

#define GUARDIAN_CONTROLLER_REF_FOR_MAX_RPM              1000

#define GUARDIAN_CONTROLLER_SERIAL_DELAY		         5000		    // us between serial transmisions to the AX3500 controller
#define GUARDIAN_CONTROLLER_ABSOLUTE_ENCODERS			 1	            // from AX3500.h
#define GUARDIAN_CONTROLLER_RELATIVE_ENCODERS			 2	            // -//-
#define GUARDIAN_CONTROLLER_MAX_ENC_INC                  1500            // maximal increment that could be read in 1 period = maximal speed

#define GUARDIAN_CONTROLLER_ENCODER_CONF                 2                // Depending on the position of both encoder (values = 1 or 2)
#define GUARDIAN_CONTROLLER_ENCODER_DIR                  1.0              // Inverts or no the read value from the encoders (vales = -1 or 1)

#define GUARDIAN_CONTROLLER_MAIN_BATTERY_CONST           0.214844        // 55/256
#define GUARDIAN_CONTROLLER_INTERNAL_VOLTS_CONST         0.111328        // 28.5/256

#define GUARDIAN_CONTROLLER_DEFAULT_P_L_DIVISOR          0.1
#define GUARDIAN_CONTROLLER_DEFAULT_P_A_DIVISOR          0.3
#define GUARDIAN_CONTROLLER_DEFAULT_I_L                  0.0	//41.0 version original
#define GUARDIAN_CONTROLLER_DEFAULT_I_A                  0.0	//25.0
#define GUARDIAN_CONTROLLER_DEFAULT_ERR_SAT_L_DIVISOR    0.350
#define GUARDIAN_CONTROLLER_DEFAULT_ERR_SAT_A_DIVISOR    0.330

// ROBOT SPEED LIMITS
#define GUARDIAN_CONTROLLER_MAX_XSPEED         	        1.5     // [m/s]  - max speed of each track = (1800rpm/25/60)*pi*0.25=0.95<1.00
#define GUARDIAN_CONTROLLER_MAX_YAWSPEED       	        0.7     // [rad/s] w = (vr-vl)/GUARDIAN_D_TRACKS_M =(0.95+0.95)/ 1.15

#define GUARDIAN_CONTROLLER_ANALOG_INPUTS               4       // Number depending on the model of the controller
#define GUARDIAN_CONTROLLER_DIGITAL_INPUTS              2       // Number depending on the model of the controller


#define ROBOTEQ_DEFAULT_ENCODER_CONF					1
#define ROBOTEQ_DEFAULT_ENCODER_DIR						-1
#define ROBOTEQ_DEFAULT_ANGULAR_DIR						1

//! Different types of control mode
enum MotorControlMode{
    MCM_OPEN_SPEED = 0,
    MCM_CLOSED_SPEED = 1,
    MCM_CLOSED_POSITION = 2
};

//! Input control modes of the AX3500
enum InputControlMode{
    ICM_RC_MODE = 10,        // Radio control
    ICM_RS232 = 11,          // Serial without watchdog
    ICM_RS232_WATCHDOG = 12, // Serial with watch dog
    ICM_ANALOG = 13          // Analog
};

//! Position calculated
typedef struct _pose {
        double vx;
        double vy;
        double va;
        double px;
        double py;
        double pa;
} pose;

//! Important data of the controller
typedef struct _data {
    float battery;              // % of battery
    float voltage;
    float voltage_internal;     // value used
    int temperature[2];         // 2 heatsink temperatures
    float analog_input[GUARDIAN_CONTROLLER_ANALOG_INPUTS];
    bool digital_output;        // 1 digital output (real value)
    bool digital_output_setvalue; // digital output (set value)
    bool digital_input[GUARDIAN_CONTROLLER_DIGITAL_INPUTS];
    int encoder_left;           // left encoder pulses
    int encoder_right;          // right encoder pulses
    int last_encoder_left;      // left encoder pulses
    int last_encoder_right;     // right encoder pulses
	double v_ref_mps;	        // current reference linear speed [mps]
	double w_ref_rads; 	        // current reference angular speed [rads]
    double actSpeedLmps;        // current speed left
    double actSpeedRmps;        // current speed right
    double desSpeedLmps;        // reference speed left
    double desSpeedRmps;        // reference speed right
    double max_linear_speed;    // maximal linear speed
    double max_angular_speed;   // maximal angular speed
    bool bMotorsEnabled;        // motors enabled or not
    float rpm_left;             // RPM motor left
    float rpm_right;            // RPM motor right
    int channel_A_ref;          // Last reference sent to channel A
    int channel_B_ref;          // Last reference sent to channel B
	int current_consumption[2];	// Consumption of each motor (A)
} roboteq_data;

//! structure to save the parameters
typedef struct pid_params{
    //! P and I constants to control the linear and angular speed
    double KpL, KpA, KiL, KiA;
    //! Applies a % reduction  to the P constant (Adjust the P with these params)
    double KpL_divisor,  KpA_divisor;
    //! Error de saturación de I
    double ErrSatIL, ErrSatIA;
    //! Applies a % reduction  to the error of saturation of I constant
    double ErrSatIL_divisor, ErrSatIA_divisor;
}pid_params;

//! Class to operate the guardian_controller servo driver
class guardian_controller: public Component {

    private:

        //! Mutex for controlling the changes and access to encoder values
        pthread_mutex_t mutex_encoders;
        //! Mutex for controlling the changes and access to encoder values
        pthread_mutex_t mutex_odometry;
		//! Object to communicate with the driver
		RoboteqDevice *roboteq;
        //! Establish encoder's data process (Absolute or Relative)
        int encoders_mode;
		//! Control mode of the robot (RS232, R/C)
		int control_mode;
		//! Odometry data
		pose robot_pose;
		//! Robot data
		roboteq_data robot_data;
		//! Error counter (just for debug)
		int err_counter;
		//! Internal error code used to select the apropiate recovery action
		int iErrorType;
		//! Saves the string to read each encoder
		char cmdEncLeft[8], cmdEncRight[8];
		//! Current motor control mode
        int motor_control_mode;
        //! Constant to convert wheel rpm to mps
        double dWheelRpmToMps;
        double dMpsToWheelRpm;
        //! Constant to convert rpm to mps
        double dRpmToWheelRpm;
        double dWheelRpmToRpm;
        //! Constant to convert rpm to guardian_controller reference
        double dRpmToRef;
        double dRefToRpm;
        //! Constant to convert mps to guardian_controller reference
        double dMpsToRef;
        double dRefToMps;
        //! linear and angular speed of the robot
        double linearSpeedMps, angularSpeedRads;
        //! pid parameters
        pid_params pidMotor;
        //! Diameter of the wheel. Used to calculate the odometry
        float fDiameterWheel;
        //! Distance between motor wheels. Used to calculate the odometry
        float fDistanceBetweenWheels;
        //! Flag active whether we want to apply the PID controller
        bool bPID;
        //! Device
        string sDevice;
        //! Channel where is mounted each encoder
        int iChannelEncLeft, iChannelEncRight;
        //! Constant applied to convert RPM to MPS, depending on the odometry type
		double dConversionFactor;
		//! Constant used to convert counts per sec to Revolutions per min
		double dCountsPerSec_To_RevPerMin;
		//!	Sets the internal encoders' configuration (values: 1, 2)
		int iEncoderConfig;
		//! Sets the internal encoders' direction (values: -1, 1)
		int iEncoderDir;
		//! Sets the internal direction for the angular speed reference (values: -1, 1)
		int iAngularSpeedDir;

    public:

        //! Parametrized public constructor
        guardian_controller(const char *device, double hz);
        //! Public destructor
        ~guardian_controller();
        //! Switches motor power on/off
        void ToggleMotorPower(bool val);
        //!	Returns robot velocity and pose
        void GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa);
        //! Returns the robot position (equivalent to get odometry)
        pose GetPose();
        //! Resets driver odometry
        void ResetOdometry();
        //! Set new odometry value (pose)
        void ModifyOdometry(  double px,  double py,  double pa );
        //! Set robot left and right track references from linear and angular values
        void SetSpeed(double lin_speed, double ang_speed);
        //! Set robot linear and angular speed limits
        void SetSpeedLimits(float lin_speed, float ang_speed);
        //! Return the robot voltage value measured internally
        float GetVoltage();
        //! Return the last value measured of encoder 'L'eft or 'R'ight
        int GetEncoder(char enc);
        //! Return the heat sink temperature measurement
        int GetTemp(int index);
        //!	Return the selected analog input measurement
        float GetAnalogInput(int index);
        //!	Set the digital output to the selected value
        void SetDigitalOutput(bool value);
        //!	Get Digital Output
        bool GetDigitalOutput();
        //! Return value of selected digital input (0..2)
        bool GetDigitalInput(int input);
        //! Returns the current input control mode (RC, RS232,  ..)
        int GetInputControlMode();
        //! Returns the current Motor control mode (MIXED, SEPARATED, ..)
        int GetMotorControlMode();
        //! Gets the percent of the total battery
        double GetBatteryPercent();
        //! Gets the rpm of each motor
        void GetRPM(float *left_rpm, float *right_rpm);
        //! Gets the channel's reference sent to the controller
        void GetChannelReference(int *a, int *b);
        //! Gets the maximum configured speeds
        void GetRobotSpeedLimits(float *lin_speed, float *ang_speed);
        //! Sets the wheels' diameter and the distance between them
        void SetMotorWheelParams(float diameter, float distance);
        //! Sets the P adjustable parameters
        void SetP(float p_linear_divisor, float p_angular_divisor);
        //! Sets the I adjustable parameters
        void SetI(float i_linear_constant, float i_angular_constant, float i_saturation_linear_error_divisor, float i_saturation_angular_error_divisor);
        //! Enables / Disables the PID controller
        void EnablePID(bool value);
        //! Sets the PID parametes
        void SetPID(pid_params new_pid);
        //! Gets current PID params
        pid_params GetPID();
        //! Gets all the data of the AX3500
        roboteq_data GetData();
        //! Return true if the motor is enabled
        bool IsMotorEnabled();
        //! Return true if the PID is enabled
        bool IsPIDEnable();
        //! Sets the conversion factor (attribute dConversionFactor)
		void SetConversionFactor(double val);
		//!	Returns the current Motor control mode as string
		const char * GetMotorControlModeString(); 
		//!	Returns the current input control mode as string
		const char * GetInputControlModeString();
		//! Gets the desired linear and angular speed
		void GetDesiredSpeed(float *linear, float *angular);
		//! Sets the internal encoders' configuration
		void SetEncoderConfig(int config, int direction, int angular_dir);
    private:
        //!Open serial ports (called by Component::Setup)
        ReturnValue Open();
        //! Closes serial port
        ReturnValue Close();
         //! Configures the component
        ReturnValue Configure();

        //!	Switches on/off the motor
        void TogleMotorPower(int val);
        //! Read battery's charge
        void ReadBatteryVoltage();
        //!	Read the internal temperature
        void ReadTemperature();
        //!	Read the control mode
        void ReadMotorControlMode();
        //!	Read the current input control mode
        void ReadInputControlMode();
        //!	Write the control mode (mixed/separated axes, open/closed loop)
        void WriteMotorControlMode(MotorControlMode mcm);
        //! Reads the analog input
        int ReadAnalogInput(int number);
        //! Read digital inputs
        int ReadDigitalInput(int number);
        //! Controller software reset
        void ResetController();
        //!	Configure controller to accept RS232 commands
        void EnterRS232Mode();
        //! Read digital inputs
        int WriteDigitalOutput(bool value);
        //!	Convert an hexadecimal value into a decimal value.
        int HexToDec(char *value, int size);
        //!	Convert an hexadecimal value into a decimal value, using Ca2 of the hexadecimal number
        int HexToDec(char *value);
        //!	Transform read analog value into temperature value
        int ValToHSTemp(int AnaValue);
        //!	Read encoders value
        int ReadEncoders();
        //! Resets the encoders counter with response confirmation
        int ResetEncoders();
        //! Calculates Delta distance (tracks)
        double CalculateDeltaDistance(double *delta_left, double *delta_right);
        //! Calculates RPM
        double CalculateRPM(double *rpm_left, double *rpm_right);
        //!	Updates robot's odometry
        void UpdateSimpleOdometry(); //tracks
        //!	Updates robot's odometry
        void UpdateOdometry();
        //! Writes motors speed references
        // OLD: void WriteMotorSpeed(double speedL, double speedR);
        void WriteMotorSpeed(int channel_a, int channel_b);
        //! AX3500 configured AB mixed, closed loop
        void SetVWRef();
        //! Component State Machine ReadyState
        void ReadyState();
        //! Actions in Failure State
        void FailureState();
        //! Component State Machine InitState
        void InitState();
        //! Read message from the device
        int ReadMsg();
        //! Calculates the % of batt depending on the voltage
        float CalculateBattery(double voltage);
        //! Configs the constants parameters
        void ConfigureConstants();

};


#endif
