/** \file ax3500.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2010
 *
 * \brief ax3500 class driver
 * Component to manage ax3500 servo driver
 * (C) 2010 Robotnik Automation, SLL
*/
#include <guardian_node/ax3500.h>
#include <string.h> 
#include <math.h>
#include <cstdlib>
#include <iostream>

using namespace std;


/*!	\fn ax3500::ax3500(const char *device, double hz)
 * 	\brief Public parmetrized constructor
*/
ax3500::ax3500(const char *device, double hz): Component(hz) {

    // Initialization for odometry mutex
    pthread_mutex_init(& mutex_odometry, NULL);
    // Initialization of encoder mutex
    pthread_mutex_init(& mutex_encoders, NULL);
    // Component name
    sComponentName.assign("ax3500");
    // Create serial port
    serial = new SerialDevice(device, AX3500_DEFAULT_BAUDRATE,
                              AX3500_DEFAULT_PARITY, AX3500_DEFAULT_DATA_SIZE, 20 ); //Creates serial device

	this->control_mode = -1; //AX3500_CONTROL_RC;		    // By default control will be Radio Control
    this->encoders_mode = AX3500_ABSOLUTE_ENCODERS;
    this->err_counter = 0;                          // Debug counts number of communication errors
    this->motor_control_mode = -1;                  // Unknown until reading

    this->robot_data.last_encoder_left = 0; this->robot_data.encoder_left = 0;
    this->robot_data.last_encoder_right= 0; this->robot_data.encoder_right = 0;
    this->iErrorType = AX3500_ERROR_NONE;
    // Default PID constants values
    this->bPID = false;                         // By default the PID controller is not applied
    this->pidMotor.KiL = AX3500_DEFAULT_I_L;                                 // I constant for linear velocity
    this->pidMotor.KiA = AX3500_DEFAULT_I_A;                                 // I constant for angular velocity
    this->pidMotor.KpL_divisor = AX3500_DEFAULT_P_L_DIVISOR;
    this->pidMotor.KpA_divisor = AX3500_DEFAULT_P_A_DIVISOR;
    this->pidMotor.ErrSatIL_divisor = AX3500_DEFAULT_ERR_SAT_L_DIVISOR;
    this->pidMotor.ErrSatIA_divisor = AX3500_DEFAULT_ERR_SAT_A_DIVISOR;
    // Sets the maximum speed by default
    this->SetSpeedLimits(AX3500_MAX_XSPEED, AX3500_MAX_YAWSPEED);
    // Default values
    this->fDistanceBetweenWheels = MOTOR_D_TRACKS_M;
    this->fDiameterWheel = MOTOR_DIAMETER_WHEEL;
	// Constant for conversion
	this->dCountsPerSec_To_RevPerMin = 60.0/MOTOR_COUNTS_PER_REV;	// Converts counts/sec into Rev/Min 
	this->dConversionFactor = 1.0;
    // Configures PID values
    ConfigureConstants();
	// Configures constants by default for encoders
  	SetEncoderConfig(AX3500_ENCODER_CONF, AX3500_ENCODER_DIR, AX3500_ANGULAR_DIR);

	// Initializing variables
	ResetOdometry();
	robot_data.battery = 0.0;              // % of battery
    robot_data.voltage = 0.0;
    robot_data.voltage_internal  = 0.0;     // value used
    robot_data.digital_output = false;        // 1 digital output (real value)
    robot_data.digital_output_setvalue = false; // digital output (set value)
    robot_data.encoder_left = 0;           // left encoder pulses
    robot_data.encoder_right = 0;          // right encoder pulses
    robot_data.last_encoder_left = 0;      // left encoder pulses
    robot_data.last_encoder_right = 0;     // right encoder pulses
	robot_data.v_ref_mps = 0.0;	        // current reference linear speed [mps]
	robot_data.w_ref_rads = 0.0; 	        // current reference angular speed [rads]
    robot_data.rpm_left = 0.0;             // RPM motor left
    robot_data.rpm_right = 0;            // RPM motor right
    linearSpeedMps = 0.0;
    angularSpeedRads = 0.0;
    robot_data.channel_A_ref = this->robot_data.channel_B_ref = 0;
}

/*!	\fn ax3500::~ax3500()
 * 	\brief Public destructor
*/
ax3500::~ax3500(){

    // Delete serial port
    if (serial!=NULL)
        delete serial;

    // Destroy odometry mutex
	pthread_mutex_destroy(& mutex_odometry );

    // Destroy encoders mutex
	pthread_mutex_destroy(& mutex_encoders );

}

/*!	\fn ReturnValue ax3500::Open()
 * 	\brief Open device
 * 	\returns ERROR
 * 	\returns OK
*/
ReturnValue ax3500::Open(){
    // Sets the same Log port and IP
    //this->serial->SetLogParameters(iLogPort, (char*)sLogIp.c_str());
	// Setup and start AX3500 device
	if(this->serial->Setup()==ERROR) {
		ROS_ERROR("ax3500::Open: Error opening serial port");
		SwitchToState(FAILURE_STATE);
		this->iErrorType = AX3500_ERROR_OPENING;
		return ERROR;
    }
    // Start AX3500 device
    if(this->serial->Start()==ERROR) {        
        ROS_ERROR("ax3500::Open: Error calling serial device Start");
        this->iErrorType = AX3500_ERROR_OPENING;
        SwitchToState(FAILURE_STATE);
        return ERROR;
    }
    ROS_DEBUG("ax3500::Open: device opened successfully");
	return OK;
}

/*!	\fn ReturnValue ax3500::Close()
 * 	\brief Closes serial port
 * 	\returns ERROR
 * 	\returns OK
*/
ReturnValue ax3500::Close(){

    if (serial!=NULL) {
        serial->Stop();
        serial->ShutDown();
    }

	return OK;
}

/*!	\fn void ax3500::ReadBatteryVoltage()
	* Read battery's charge.
	* Send the query to the controller.
	* Command = ?e o ?E
*/
void ax3500::ReadBatteryVoltage(){
	char cRecBuffer[BUFFER]="";
	memset(cRecBuffer, 0, BUFFER);
	int written_bytes=0;
	if(serial->WritePort((char*)"?e\r",&written_bytes, 3) != OK){
		ROS_ERROR("ax3500::ReadBatteryVoltage: Error sending message");
    }

	// read response from AX3500
	char c;
	char buf[8];
	int n=0;			//Number of received bytes
	int j=0;			//Tokens' counter
	int i=0;
	memset(cRecBuffer, 0, BUFFER);
	bool bBat=false, bBat1=false, bEndBat=false;

	memset(buf,0,8);
	while (!( bEndBat ) && j<100000) {	//
		j++;
        serial->ReadPort(&c,&n, 1);
		if(n>0){
			if (c!='\r') {
				buf[i]=c;
				i++;

            }else { // return
				if (i>0) {
					if (bBat1) { // Process AX3500 internal voltage
						robot_data.voltage_internal = (float)HexToDec(buf,2)* AX3500_INTERNAL_VOLTS_CONST;//0.111328125;
						bEndBat = true;
						bBat1 = false;
					} else
					if (bBat) { // Process batt voltage
						robot_data.voltage = (float)HexToDec(buf,2)*AX3500_MAIN_BATTERY_CONST;//0.210347828;//0.21484375;
						// Test: only calculates the percentage if the robot is stopped
						if( (robot_pose.va == 0.0) && (robot_pose.vx == 0.0) && (robot_pose.vy == 0.0))
							robot_data.battery = CalculateBattery(robot_data.voltage);
						bBat = false;
						bBat1 = true;
					} else if (!strcmp(buf,"?e")) bBat = true;
                }
                i=0;
                memset(buf,0,8);
			}
		}
	}

	if(j>=100000){
		ROS_ERROR("ax3500::ReadBatteryVoltage: ErrCount: %d\n",this->err_counter);
		this->err_counter++;
		usleep(200 * AX3500_SERIAL_DELAY);
	} else {
		usleep(AX3500_SERIAL_DELAY);
	}
}

/*!	\fn void void ax3500::ReadTemperature()
 * 	\brief Read the internal temperature.
 *  Send the query to the controller.
 *  Command = ?m o ?M
*/
void ax3500::ReadTemperature(){

    int written_bytes=0;
	if(serial->WritePort((char*)"?M\r",&written_bytes,3) != OK){
		//puts("ax3500::ReadTemperature: Error sending message");
		ROS_ERROR("ax3500::ReadTemperature: Error sending message");
    }

    usleep(AX3500_SERIAL_DELAY);

	// read response from AX3500 - temperatures
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
                    robot_data.temperature[0]=ValToHSTemp(HexToDec(&buf[2],2));
                    robot_data.temperature[1]=ValToHSTemp(HexToDec(&buf[4],2));
                    //printf("Temperatures: (1)%d    (2)%d\n ", robot_data.temperature[0], robot_data.temperature[1] );
                }
            }
        }
	}

	if(j==50000){
		ROS_ERROR("ax3500::ReadTemperature: {%d} ErrData ",j);
		this->err_counter++;
    }
}

/*!	\fn void void ax3500::ReadControlMode()
 * 	\brief Read the control mode
 *  Send the query to the controller.
 *  Command = ^01
*/
void ax3500::ReadMotorControlMode(){
    bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8], mode[3]="\0";
	memset(buf,0,8);
    int written_bytes=0;

	if(serial->WritePort((char*)"^01\r",&written_bytes, 4) != OK){
		ROS_ERROR("ax3500::ReadMotorControlMode: Error sending message");
    }
	usleep(AX3500_SERIAL_DELAY);

	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
                }
            }
        }
	}

    if(bEnd){
        // Copy the value of the mode from the response
        mode[0] = buf[3];
        mode[1] = buf[4];
        //printf("ax3500::ReadMotorControlMode: mode = %s\n", mode);
        if(!strcmp(mode, "00")){
            motor_control_mode = MCM_SEPARATED_OPEN;
           // printf("ax3500::ReadMotorControlMode: MCM_SEPARATED_OPEN\n");
        }else if(!strcmp(mode, "C5")){
            motor_control_mode = MCM_MIXED_CLOSED;
            //printf("ax3500::ReadMotorControlMode: MCM_MIXED_CLOSED\n");
        }else if(!strcmp(mode, "01")){
            motor_control_mode = MCM_MIXED_OPEN;
            //printf("ax3500::ReadMotorControlMode: MCM_MIXED_OPEN\n");
        }
    }
	if(j> 50000){
		ROS_ERROR("ax3500::ReadMotorControlMode: {%d} ErrData ",j);        
	}
}

/*!	\fn void void ax3500::ReadInputControlMode()
 * 	\brief Read the current input control mode
 *  Send the query to the controller and read the response
 *  Command = ^00
*/
void ax3500::ReadInputControlMode(){
    bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8], mode[3]="\0";
	memset(buf,0,8);
    int written_bytes=0;

	if(serial->WritePort((char*)"^00\r",&written_bytes, 4) != OK){
		ROS_ERROR("ax3500::ReadInputControlMode: Error sending message");
    }
	usleep(AX3500_SERIAL_DELAY);

	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
				    //printf("ax3500::InputControlMode = %s\n", buf);
                }
            }
        }
	}

    if(bEnd){
        // Copy the value of the mode from the response
        mode[0] = buf[3];
        mode[1] = buf[4];
        //printf("ax3500::ReadMotorControlMode: mode = %s\n", mode);
        if(!strcmp(mode, "00")){
            control_mode = ICM_RC_MODE;
            //printf("ax3500::ReadMotorControlMode: ICM_RC_MODE\n");
        }else if(!strcmp(mode, "01")){
            control_mode = ICM_RS232;
            //printf("ax3500::ReadMotorControlMode: ICM_RS232\n");
        }else if(!strcmp(mode, "02")){
            control_mode = ICM_RS232_WATCHDOG;
            //printf("ax3500::ReadMotorControlMode: ICM_RS232_WATCHDOG\n");
        }else if(!strcmp(mode, "03")){
            control_mode = ICM_ANALOG;
            //printf("ax3500::ReadMotorControlMode: ICM_RS232_WATCHDOG\n");
        }
    }
	if(j> 50000){
		ROS_ERROR("ax3500::ReadInputControlMode: {%d} ErrData ",j);
	}

}

/*!	\fn void ax3500::WriteMotorControlMode(MotorControlMode mcm)
 * 	\brief Set the control mode
 *  Send the query to the controller.
 *  Command = ^01 C5 : mixed mode - closed loop
              ^01 01 : mixed mode - open loop
              ^01 00 : separate mode - open loop
*/
void ax3500::WriteMotorControlMode(MotorControlMode mcm){
    int written_bytes=0;
    char cCommand[8] = "\0";
    string sMode("");

    // Create the string depending on the selected command
    switch(mcm){
        case MCM_MIXED_OPEN:
            strcpy(cCommand, "^01 01\r");
            sMode.assign("Mixed mode + open loop");
        break;
        case MCM_MIXED_CLOSED:
            strcpy(cCommand, "^01 C5\r");
            sMode.assign("Mixed mode + closed loop");
        break;
        case MCM_SEPARATED_OPEN:
            strcpy(cCommand, "^01 00\r");
            sMode.assign("Separated mode + open loop");
        break;
        case MCM_SEPARATED_CLOSED:
            strcpy(cCommand, "^01 C4\r");
            sMode.assign("Separated mode + open loop");
        break;
    }
	
    // Sends the command to configure the mode
    if(serial->WritePort(cCommand, &written_bytes, 7) != OK){ //separate mode, open loop
        ROS_ERROR("ax3500::WriteMotorControlMode: Error setting %s", sMode.c_str());
	    return;
    }
	usleep(AX3500_SERIAL_DELAY);
	// read response from AX3500 - command !
	bool bEnd=false;
	int j=0,n=0;
	char c;
	while (!( bEnd ) && j<50000) {	// wait until read '+'
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				if (c=='+') {
                  bEnd=true;
                }
            }
        }
	}

	if(j>=50000){
	    ROS_ERROR("ax3500::WriteMotorControlMode: Error reading response");
		//printf("{%d} ErrData ",j);
		this->err_counter++;
		return;
    }

    // Sends the command to reset the device
	if(serial->WritePort((char*)"^FF\r",&written_bytes, 4) != OK){
		ROS_ERROR("ax3500::WriteMotorControlMode: Error reseting the device after configure %s", sMode.c_str());
		return;
    }

	usleep(AX3500_SERIAL_DELAY);
    bEnd = false;
    j= n = 0;
	while (!( bEnd ) && j<50000) {	// wait until read '+'
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				if (c=='+') {
                  bEnd=true;
                }
            }
        }
	}

	if(j>=50000){
	    ROS_ERROR("ax3500::WriteMotorControlMode: Error reading response after reseting");
		this->err_counter++;
		return;
    }

	ROS_DEBUG("ax3500::WriteControlMode: AX3500 configured succesfully on %s", sMode.c_str());
}

/*!	\fn void ax3500::ReadAnalogInputs()
 * 	\brief Read analog inputs
 * 	Send the query to the controller.
 * 	Command = ?p o ?P
*/
void ax3500::ReadAnalogInputs(){

    // send request to AX3500, analog inputs
    int written_bytes=0;
	if(serial->WritePort((char*)"?P\r",&written_bytes, 3) != OK){
		ROS_ERROR("ax3500::ReadAnalogInputs: Error sending message");
    }
	usleep(AX3500_SERIAL_DELAY);

	// read response from AX3500 - analog input
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
                    int aux1=HexToDec(&buf[2],2);
                    if(aux1==255) aux1=-127;
                    else if(aux1>127) aux1=127-aux1;
                    int aux2= HexToDec(&buf[4],2);
                    if(aux2==255) aux2=-127;
                    else if(aux2 > 127) aux2 = 127 - aux2;
                    //-127=0V, 0=2.5V, 127= 5V
                    // Lineal interpolation-> m= 0.019685039, b= 2.5
                    float m=0.019685039;
                    float b=2.5;
                    robot_data.analog_input[0] = m*(float) aux1+b;
                    robot_data.analog_input[1] = m*(float) aux2+b;
                    //printf("Anlog Inputs: (1)%f    (2)%f\n ", robot_data.analog_input[0], robot_data.analog_input[1]);
                }
            }
        }
	}

	if(j>=50000){
		ROS_ERROR("ax3500::ReadAnalogInputs: {%d} ErrData ",j);
		this->err_counter++;
    }
}

/*!	\fn void ax3500::ReadDigitalInputs()
 * 	\brief Read digital inputs
 * 	Send the query to the controller.
 * 	Command = ?i o ?I
*/
void ax3500::ReadDigitalInputs(){

    int written_bytes=0;
	if(serial->WritePort((char*)"?I\r",&written_bytes, 3) != OK) {
		ROS_ERROR("ax3500::ReadDigitalInputs: Error sending message");
    }

	usleep(AX3500_SERIAL_DELAY);

	// read response from AX3500 - analog input
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8];
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=8) {
				    bEnd=true;
                    //int aux1=HexToDec(&buf[2],2);
                    //int aux2=HexToDec(&buf[4],2);
                    //int aux3=HexToDec(&buf[6],2);
                    robot_data.digital_input[0] = (bool) HexToDec(&buf[2],2);
                    robot_data.digital_input[1] = (bool) HexToDec(&buf[4],2);
                    robot_data.digital_input[2] = (bool) HexToDec(&buf[6],2);
                    //printf("Anlog Inputs: (1)%f    (2)%f\n ", robot_data.analog_input[0], robot_data.analog_input[1]);
                }
            }
        }
	}

	if(j==50000){
		ROS_ERROR("ax3500::ReadDigitalInputs:: {%d} ErrData ",j);
		this->err_counter++;
        }

}


/*!	\fn void ax3500::ResetController()
 * 	\brief Allows the controller to be reset in the same manner as if the reset button
 * 	was pressed.
 *  Command = %rrrrrr
*/
void ax3500::ResetController(){

	int written_bytes=0;

	//WriteMotorSpeed(0.0,0.0); // Stop Motors

	ToggleMotorPower(false);  // Set flag to motors disabled

	if(serial->WritePort((char*)"%rrrrrr\r",&written_bytes, 8) != OK){
		ROS_ERROR("guardian::ResetController: Error sending message");
    }
	usleep(AX3500_SERIAL_DELAY);
}

/*!	\fn void ax3500::EnterRS232Mode()
 * 	\brief If the controller is configured in R/C or Analog mode, it will not be able to accept and recognize
 * 	RS232 commands immediately. It will enter the serial mode after it has received 10 continuous
 * 	“Enter” (Carriage Return) characters
*/
void ax3500::EnterRS232Mode(){
    int written_bytes=0;
	for(int i=0; i < 10; i++)	{
		if(serial->WritePort((char*)"\r",&written_bytes, 1) != OK)	{
			ROS_ERROR("ax3500::EnterRS232Mode: Error sending message");
            }
		usleep(2*AX3500_SERIAL_DELAY);
	}
	//serial->WritePort("^01 00\r", &written_bytes, 7 );
	//usleep(2*AX3500_SERIAL_DELAY);
	ROS_DEBUG("ax3500: Entering RS232 mode");
}

/*!	\fn int ax3500::WriteDigitalOutput(bool value)
 * 	\brief Set the digital output into the selected value.
 *		 Initial configuration: output 1..4 managed by usbdux device
 *								output 5 managed by AX3500 board
 * 	\param bit as int, output number from 1 to DIGITAL_OUTPUTS
 * 	\param value as int, output value
 * 	\return ERROR if the output can't be set
 * 	\return OK
*/
int ax3500::WriteDigitalOutput(bool value){

    int written_bytes=0;

    if(value) { // AX3500 Command= !C (true) or !c (false)
        if(serial->WritePort((char*)"!C\r",&written_bytes, 3) != OK){
            ROS_ERROR("ax3500::WriteDigitalOutput: Error sending message");
            return ERROR;
        }
    }else {
        if(serial->WritePort((char*)"!c\r",&written_bytes, 3) != OK){
            ROS_ERROR("ax3500::WriteDigitalOutput: Error sending message");
            return ERROR;
        }
    }

	// read response from AX3500 - command !
	bool bEnd=false;
	int j=0,n=0;
	char c;
	while (!( bEnd ) && j<5000) {	// wait until read '+'
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				if (c=='+') {
                  bEnd=true;
                  robot_data.digital_output = value;
                }
            }
        }
	}

	if(j>=5000){
		ROS_ERROR("ax3500::WriteDigitalOutputs: {%d} ErrData ",j);
		this->err_counter++;
    }

	return OK;
}

/*!	\fn int ax3500::HexToDec(char *value, int size)
	* Convert an hexadecimal value into a decimal value.
	* @param value as char *, is an string with the value
	* @param size as integer, is the size of the string
	* return -1 if error
*/
int ax3500::HexToDec(char *value, int size){
	int aux = 0;
	int aux_value;

	for(int i=0; i<size; i++) {
		//printf("value[%d]=%c   ", i, value[i]);
		aux_value = 0;
		switch(value[i]) { //transforms each character into an integer
			case '0':
				aux_value = 0;
			break;
			case '1':
				aux_value = 1;
			break;
			case '2':
				aux_value = 2;
			break;
			case '3':
				aux_value = 3;
			break;
			case '4':
				aux_value = 4;
			break;
			case '5':
				aux_value = 5;
			break;
			case '6':
				aux_value = 6;
			break;
			case '7':
				aux_value = 7;
			break;
			case '8':
				aux_value = 8;
			break;
			case '9':
				aux_value = 9;
			break;
			case 'A':
				aux_value = 10;
			break;
			case 'B':
				aux_value = 11;
			break;
			case 'C':
				aux_value = 12;
			break;
			case 'D':
				aux_value = 13;
			break;
			case 'E':
				aux_value = 14;
			break;
			case 'F':
				aux_value = 15;
			break;
			default:
				return -1;
			break;
		}

		aux+= (int)(aux_value*pow(16,(size-1-i)));		// 0x71 = 7*16^1 + 1*16^0 = 113
	}

	return aux;
}

/*!	\fn int ax3500::HexToDec(char *value)
	* Convert an hexadecimal value into a decimal value, using Ca2 of the hexadecimal number
	* @param value as char *, is an string with the value
	* return converted value
*/
int ax3500::HexToDec(char *value){
	int aux = 0;
	int aux_value=0;
	int signal = 1;
	int size = 0;

	for(int i=0; i<8; i++) {//Calculate the size of the number
		switch(value[i]) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case 'A':
			case 'B':
			case 'C':
			case 'D':
			case 'E':
			case 'F':
				size++;
			break;
			default:
				i=8;
			break;
		}
	}

	// Is a negative number? More Significant Bit == 1 -> Negative
	if((value[0]=='F') || (value[0]=='E') || (value[0]=='D') || (value[0]=='C') || (value[0]=='B')
		|| (value[0]=='A') || (value[0]=='9') || (value[0]=='8'))
		signal = -1;

	if(signal==-1) { //If it's negative
		for(int i=0; i<size; i++) { //Apply Ca1
			aux_value = 0;
			switch(value[i]) { //transforms each character into an integer
				case '0':
					aux_value = 15;
				break;
				case '1':
					aux_value = 14;
				break;
				case '2':
					aux_value = 13;
				break;
				case '3':
					aux_value = 12;
				break;
				case '4':
					aux_value = 11;
				break;
				case '5':
					aux_value = 10;
				break;
				case '6':
					aux_value = 9;
				break;
				case '7':
					aux_value = 8;
				break;
				case '8':
					aux_value = 7;
				break;
				case '9':
					aux_value = 6;
				break;
				case 'A':
					aux_value = 5;
				break;
				case 'B':
					aux_value = 4;
				break;
				case 'C':
					aux_value = 3;
				break;
				case 'D':
					aux_value = 2;
				break;
				case 'E':
					aux_value = 1;
				break;
				case 'F':
					aux_value = 0;
				break;
			}

			aux+=(int)(aux_value*pow(16,(size-1-i)));		// 0x71 = 7*16^1 + 1*16^0 = 113
		}
		aux+=1;//Ca1 + 1 = Ca2
		aux=-aux;
	}else {	//Positive
		for(int i=0; i<size; i++) {
			//printf("value[%d]=%c   ", i, value[i]);
			aux_value = 0;
			switch(value[i]) {//transforms each character into an integer
				case '0':
					aux_value = 0;
				break;
				case '1':
					aux_value = 1;
				break;
				case '2':
					aux_value = 2;
				break;
				case '3':
					aux_value = 3;
				break;
				case '4':
					aux_value = 4;
				break;
				case '5':
					aux_value = 5;
				break;
				case '6':
					aux_value = 6;
				break;
				case '7':
					aux_value = 7;
				break;
				case '8':
					aux_value = 8;
				break;
				case '9':
					aux_value = 9;
				break;
				case 'A':
					aux_value = 10;
				break;
				case 'B':
					aux_value = 11;
				break;
				case 'C':
					aux_value = 12;
				break;
				case 'D':
					aux_value = 13;
				break;
				case 'E':
					aux_value = 14;
				break;
				case 'F':
					aux_value = 15;
				break;
			}
			aux+= (int)(aux_value*pow(16,(size-1-i)));		// 0x71 = 7*16^1 + 1*16^0 = 113
		}
	}

	return aux;
}

/*!	\fn int ax3500::ValToHSTemp(int AnaValue)
	* Transform read analog value into temperature value
*/
int ax3500::ValToHSTemp(int AnaValue){
	// Interpolation table. Analog readings at -40 to 150 oC, in 5o intervals
	int TempTable[39] ={248, 246, 243, 240, 235, 230, 224, 217, 208, 199, 188, 177,
						165, 153, 140, 128, 116, 104,93, 83, 74, 65, 58, 51, 45, 40, 35, 31, 27, 24, 21,
						19, 17, 15, 13, 12, 11, 9, 8};
	int LoTemp, HiTemp, lobound, hibound, temp, i;
	i = 38;

	while (TempTable[i] < AnaValue && i > 0) i--;

    if (i < 0) i = 0;

    if (i == 38)
        return 150;
    else {
        LoTemp = i * 5 - 40;
        HiTemp = LoTemp + 5;
        lobound = TempTable[i];
        hibound = TempTable[i+1];
        temp = LoTemp + (5 * ((AnaValue - lobound)*100/ (hibound - lobound)))/100;
        return temp;
    }
}

/*!	\fn void ax3500::ReadEncoders()
	* Read encoders value
*/
int ax3500::ReadEncoders(){
	int encL=0,encR=0;
	char c;
	char cRecBuffer[BUFFER]="";
	char buf[8];
	int n=0;			//Number of received bytes
	int j=0;			//Tokens' counter
	int k=0;
	int i=0;
	memset(cRecBuffer, 0, BUFFER);
	bool bQ0=false, bQ1=false, bEndQ0=false, bEndQ1=false;
    int written_bytes=0;
   // char cmd1[8];
   // char cmd2[8];

	// send request to AX3500, right encoder
    if(serial->WritePort(cmdEncRight, &written_bytes, 4) != OK) {
        ROS_ERROR("ax3500::ReadEncoders: Error sending message for encoder right");
    }
	usleep(AX3500_SERIAL_DELAY);

	// read response from AX3500 - encoder0
	memset(buf,0,8);
	while (!( bEndQ0 ) && j<50000) {	// wait until read enc0 - Q0
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
                }
			else { // return
				if (i>0) {
					if (bQ0) { // Process q0 data
						pthread_mutex_lock(&mutex_encoders);
							encR = HexToDec(buf);
							this->robot_data.encoder_right = iEncoderDir * encR;//-HexToDec(buf);
						pthread_mutex_unlock(&mutex_encoders);
						bEndQ0 = true;
						bQ0 = false;
					} else if ( !strcmp(buf,"?q0") || !strcmp(buf,"?q1") || !strcmp(buf,"?q4") || !strcmp(buf,"?q5") )
                        bQ0 = true;
				}
			i=0;
			memset(buf,0,5);
            }
        }
    }
	if(j>=50000){
		ROS_ERROR("ax3500::ReadEncoders:(Right) {%d} ErrData ",j);
		this->err_counter++;
    }

	
	if(serial->WritePort(cmdEncLeft, &written_bytes, 4) != OK) {
		ROS_ERROR("guardian::ReadEncoders: Error sending message for encoder left");
    }
	usleep(AX3500_SERIAL_DELAY);

	// read response from AX3500 - encoder1
	n=0; j=0; k=0; i=0;

	memset(buf,0,8);
	while (!( bEndQ1 ) && j<50000) {	// wait until read enc1 - Q1
		j++;
        serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received

			if (c!='\r') {
				buf[i]=c;
				i++;
                         }
			else { // return
				if (i>0) {
					if (bQ1) { // Process q1 data
						pthread_mutex_lock(&mutex_encoders);
							encL = HexToDec(buf);
							this->robot_data.encoder_left = iEncoderDir * encL;
						pthread_mutex_unlock(&mutex_encoders);
						bEndQ1 = true;
						bQ1 = false;
					} else if ( !strcmp(buf,"?q0") || !strcmp(buf,"?q1") || !strcmp(buf,"?q4") || !strcmp(buf,"?q5") )
                        bQ1 = true;
				}
			i=0;
			memset(buf,0,5);
			}
		}
	}

	if(j>=50000){
		ROS_ERROR("ax3500::ReadEncoders: (Left) {%d} ErrData ",j);
		this->err_counter++;
	}

	if (bEndQ0 && bEndQ1)
		return OK;
	else
		return ERROR;
}

/*! \fn void ax3500::ResetEncoders()
 *  \brief Resets the encoders counter with response confirmation
 *	\return OK
 *	\return ERROR
*/
int ax3500::ResetEncoders(){
	char cSendBuffer[5]="!q2\r";
	int n = 0, j = 0;
	char c;
	int counts = 200;
	int timer = 2*AX3500_SERIAL_DELAY;
	bool bEnd = false;
	int attempts = 0, max_attempts = 5;
	int written_bytes=0;

	// Send command !q2
	while(!bEnd && (attempts < max_attempts)){	// Todos los mensajes de configuración se repetirán hasta que se reciban todas las respuestas de validación (+)

		if(serial->WritePort(cSendBuffer, &written_bytes, 5) != OK) {
			ROS_ERROR("ax3500::ResetEncoders: Error sending %s to the controller", cSendBuffer);
			return ERROR;
		}
		usleep(timer);

		j=0;
		serial->ReadPort(&c, &n, 1);
		while(c!='+' && j < counts){	//Wait for confirmation
			j++;
			serial->ReadPort(&c, &n, 1);
            }
		//if(c!= '\r') // printf("ax3500::ResetEncoders: c= %c, counts=%d\n", c, j);
		if(c =='+'){
			ROS_DEBUG("ax3500::ResetEncoders: %s command OK", cSendBuffer);			
			bEnd = true;
		}
		attempts++;
	}

	if(attempts >= max_attempts){
		ROS_ERROR( "ax3500::ResetEncoders: Error: %d attempts to reset the encoders", attempts);
		
		return ERROR;
	}

	pthread_mutex_lock(&mutex_encoders);
		this->robot_data.encoder_left = 0;
		this->robot_data.encoder_right = 0;
		this->robot_data.last_encoder_left = 0;
        this->robot_data.last_encoder_right = 0;
	pthread_mutex_unlock(&mutex_encoders);

	return OK;
}


/*!	\fn double ax3500::CalculateRPM()
	* Calculates RPM
	* @return sample period in seconds
*/
double ax3500::CalculateRPM(double *rpm_left, double *rpm_right){
	static bool first = true;
	static struct timespec now, last;
	int inc_left_counts = 0, inc_right_counts = 0;
	//double secs=0.0;
	double secs= 1.0 / threadData.dRealHz;
	int diff= 0;
	double rev_per_min_left = 0.0, rev_per_min_right=0.0;


	// ABSOLUTE ENCODERS
	if(this->encoders_mode == AX3500_ABSOLUTE_ENCODERS){
        pthread_mutex_lock(&mutex_encoders);
            inc_left_counts = robot_data.encoder_left - robot_data.last_encoder_left;
            inc_right_counts = robot_data.encoder_right - robot_data.last_encoder_right;
            robot_data.last_encoder_left = robot_data.encoder_left;
            robot_data.last_encoder_right = robot_data.encoder_right;
        pthread_mutex_unlock(&mutex_encoders);
        }
    else {
    // RELATIVE ENCODERS
        pthread_mutex_lock(&mutex_encoders);

            inc_left_counts = robot_data.encoder_left;
            inc_right_counts = robot_data.encoder_right;

			if(abs(inc_left_counts) > AX3500_MAX_ENC_INC) {	// Check wrong data
                ROS_ERROR("ax3500::CalculateRPM:Relative: error reading left encoder: inc = %d", inc_left_counts);
				inc_left_counts = robot_data.last_encoder_left;
			}else{
				robot_data.last_encoder_left = robot_data.encoder_left;
            }

			if(abs(inc_right_counts) > AX3500_MAX_ENC_INC){	//En caso de que llegase un dato anómalo
				ROS_ERROR("ax3500::CalculateRPM:Relative: error reading right encoder: inc = %d", inc_right_counts);
				inc_right_counts = robot_data.last_encoder_right;
			}else{
				robot_data.last_encoder_right = robot_data.encoder_right;
			}

			robot_data.encoder_left = 0;
			robot_data.encoder_right = 0;

        pthread_mutex_unlock(&mutex_encoders);
        }

	clock_gettime(GetClock(), &now);

	/*if(first) {	//First call
		first = false;
		last= now;
		return -1.0;
        }

	tsnorm(&now);
	tsnorm(&last);

	diff = calcdiff(now,last);
	secs = (double)diff/(double)USEC_PER_SEC;*/
	//	Calcs rev per min for each motor
	*rpm_left= rev_per_min_left = (inc_left_counts/secs)*dCountsPerSec_To_RevPerMin;
	*rpm_right = rev_per_min_right = (inc_right_counts/secs)*dCountsPerSec_To_RevPerMin;
	//sprintf(aux, "ax3500::CalculateRPM: Rpm left = %lf(inc encoder = %d)\t Rpm Right = %lf(inc encoder = %d)\t time: diff= %lf secs\n", rev_per_min_left, inc_left_counts,rev_per_min_right,inc_right_counts,secs);

	//last= now;

	return secs;
}


/*!	\fn void ax3500::UpdateOdometry()
	* Updates robot's odometry
*/
void ax3500::UpdateOdometry(){
	double fVelocityLeftRpm=0.0;
	double fVelocityRightRpm=0.0;
	double v_left_mps, v_right_mps;
	double fSamplePeriod = 0.0; // Default sample period

	fSamplePeriod= CalculateRPM(&fVelocityLeftRpm,&fVelocityRightRpm);

	//if(fSamplePeriod < 0.0)
	//	return;

	// Convert velocities from rpm to m/s
	/*v_left_mps = fVelocityLeftRpm * dRpmToMps;
	v_right_mps = fVelocityRightRpm * dRpmToMps;
	linearSpeedMps = (v_right_mps + v_left_mps) / 2.0;			   		    // m/s
	angularSpeedRads = (v_right_mps - v_left_mps) / fDistanceBetweenWheels;    //rad/s*/


	// Convert velocities from rpm to m/s
	v_left_mps = fVelocityLeftRpm * dRpmToWheelRpm * dWheelRpmToMps;
	v_right_mps = fVelocityRightRpm * dRpmToWheelRpm * dWheelRpmToMps;
	linearSpeedMps = (v_right_mps + v_left_mps) / 2.0;			   		    // m/s
	angularSpeedRads = (v_right_mps - v_left_mps) / fDistanceBetweenWheels;    //rad/s

	
	pthread_mutex_lock(&mutex_odometry);

        //Left and right track velocities
        robot_data.actSpeedLmps = v_left_mps;
        robot_data.actSpeedRmps = v_right_mps;

		//Velocity
		robot_pose.va = angularSpeedRads;
		robot_pose.vx = linearSpeedMps *  cos(robot_pose.pa);
		robot_pose.vy = linearSpeedMps *  sin(robot_pose.pa);

		//Position
		robot_pose.pa += angularSpeedRads * fSamplePeriod;
		radnorm(&robot_pose.pa);
		robot_pose.px += robot_pose.vx * fSamplePeriod;
		robot_pose.py += robot_pose.vy * fSamplePeriod;

		robot_data.rpm_right = (float)fVelocityRightRpm;
		robot_data.rpm_left  = (float)fVelocityLeftRpm;

	pthread_mutex_unlock(&mutex_odometry);

	//printf("ax3500::UpdateOdometry: mpsLeft = %lf, mpsRight= %lf, LinearSpeed=%lf, AngularSpeed=%lf, X=%lf, Y=%lf, Orientation=%lf\n", v_left_mps, v_right_mps,
	//	linearSpeedMps, RTOD(angularSpeedRads), robot_pose.px, robot_pose.py, robot_pose.pa);
}

/*!\fn void ax3500::SetVWRef()
 * AX3500 state space control of V:Linear speed and W:Angular speed
 * AX3500 configured AB mixed, closed loop
*/
void ax3500::SetVWRef()
{
    int channelA = 0, channelB = 0;
    int uv = 0, uw = 0;
    static double err_lin_i = 0.0, err_ang_i = 0.0;
    double err_lin = robot_data.v_ref_mps  - linearSpeedMps;
    double err_ang = robot_data.w_ref_rads - angularSpeedRads;
   //
   // IF we want software PID control ...
    if(bPID){

        err_lin_i += err_lin;
        err_ang_i += err_ang;

        // Checks the
        if (err_lin_i > pidMotor.ErrSatIL)
            err_lin_i = pidMotor.ErrSatIL;
        else if (err_lin_i < -pidMotor.ErrSatIL) 
            err_lin_i = -pidMotor.ErrSatIL;

        if (err_ang_i > pidMotor.ErrSatIA) err_ang_i = pidMotor.ErrSatIA;
        else if (err_ang_i < -pidMotor.ErrSatIA) err_ang_i = -pidMotor.ErrSatIA;

        uv = (int) (pidMotor.KpL * err_lin + pidMotor.KiL * err_lin_i);
        uw =  (int) (pidMotor.KpA  * err_ang + pidMotor.KiA * err_ang_i);
    }
    // Motor Mixed Mode
   // if((motor_control_mode == MCM_MIXED_CLOSED) || (motor_control_mode == MCM_MIXED_CLOSED)){
        // Theoric calc of the ref for the desired speed
        //channelB = (int) (robot_data.v_ref_mps * 1.1 * dMpsToRef + uv);
        channelB = (int) iEncoderDir * (robot_data.v_ref_mps * dMpsToRef + uv);
        channelA = (int) iAngularSpeedDir * (robot_data.w_ref_rads * (dMpsToRef * fDistanceBetweenWheels) * 0.5 + uw );
    //}
   // if(channelB =
   //cout << "UV = " <<pidMotor.KpL  << " x " << err_lin << " + " << err_lin_i << endl;
   // cout << "Channel A = " << channelA << " (uw = "<< uw <<"), Channel B = " << channelB << ", (uv = " << uv << ") " << endl;
    //cout << "uv = " << KpL << " * " << err_lin << " + " << KiL << " * " << err_lin_i << endl;
    //cout << "Error lin = " << err_lin << " = " << robot_data.v_ref_mps << " - " << linearSpeedMps << endl;
    
    WriteMotorSpeed(channelA, channelB);

}

/*! \fn void ax3500::WriteMotorSpeed( angular_speed or speedL, linear_speed or speedR )
 * 	\brief Sets motor speed input variables are different variables according to AX3500 cfg.
*/
void ax3500::WriteMotorSpeed(int channel_a, int channel_b){
    int j=0, n = 0;
	char c;
	char cSendBuffer[6]="",cSendBuffer2[6]="";
	int wait_cycles = 50000;

	//Control max speed
	if((channel_a < 0) && (channel_a < -AX3500_MOTOR_DEF_MAX_SPEED) )
		channel_a = -AX3500_MOTOR_DEF_MAX_SPEED;
	else if( (channel_a > 0.0) && (channel_a > AX3500_MOTOR_DEF_MAX_SPEED) )
		channel_a = AX3500_MOTOR_DEF_MAX_SPEED;

	if((channel_b < 0.0) && (channel_b < -AX3500_MOTOR_DEF_MAX_SPEED) )
		channel_b = -AX3500_MOTOR_DEF_MAX_SPEED;
	else if( (channel_b > 0.0) && (channel_b > AX3500_MOTOR_DEF_MAX_SPEED) )
		channel_b = AX3500_MOTOR_DEF_MAX_SPEED;

	if(channel_a >= 0.0)	{
		sprintf(cSendBuffer,"!a%02x\r", channel_a);
	}else{
		sprintf(cSendBuffer,"!A%02x\r", abs(channel_a));
	}

	if(channel_b >= 0.0)	{
		sprintf(cSendBuffer2,"!b%02x\r", channel_b);
	}else{
		sprintf(cSendBuffer2,"!B%02x\r", abs(channel_b));
	}
   // cout << "Ax3500: Channel A = " << channel_a << ", Channel B = " << channel_b << endl;
    //Sends the values
    int written_bytes=0;
	if(serial->WritePort(cSendBuffer, &written_bytes, 5)!=OK){
		ROS_ERROR("ax3500::WriteMotorSpeed: Error sending message");
	}
	usleep(AX3500_SERIAL_DELAY);


    serial->ReadPort(&c, &n, 1);
    while(c!='+' && j < wait_cycles){	//Wait for confirmation
        j++;
        serial->ReadPort(&c, &n, 1);
    }
    if(c !='+'){
        ROS_ERROR("ax3500::WriteMotorSpeed: Error on %s command. No response", cSendBuffer);
    }
	//memset(cRecBuffer, 0, BUFFER);

	if(serial->WritePort(cSendBuffer2, &written_bytes, 5)!=OK){
		ROS_ERROR("ax3500::WriteMotorSpeed: Error sending message");
    }
	usleep(AX3500_SERIAL_DELAY);
    j = n = 0;
    serial->ReadPort(&c, &n, 1);
    while(c!='+' && j < wait_cycles){	//Wait for confirmation
        j++;
        serial->ReadPort(&c, &n, 1);
    }
    if(c !='+'){
        ROS_ERROR("ax3500::WriteMotorSpeed:Error on %s command. No response", cSendBuffer2);
    }
    // Saves the last references sent to the channels
    robot_data.channel_A_ref = channel_a;
    robot_data.channel_B_ref = channel_b;
}

/*! \fn void ax3500::InitState()
   * Component State Machine InitState
   * Periodic actions at component hz
*/
void ax3500::InitState()
{
    // Enter serial communication mode
    EnterRS232Mode();

    //WriteMotorControlMode(MCM_MIXED_CLOSED);
    //WriteMotorControlMode(MCM_SEPARATED_OPEN);

    // Reset Encoders (avoid initial odometry error due to wrong initialization
    if (ResetEncoders()!=OK) {
        this->iErrorType=AX3500_ERROR_SERIALCOMM;
        SwitchToState(FAILURE_STATE);
        return;
    } else {
        this->iErrorType=AX3500_ERROR_NONE;
        sleep(1);
        SwitchToState(READY_STATE);
	}

}

/*! \fn void ax3500::ReadyState()
   * Component State Machine ReadyState
   * Periodic actions at component hz
*/
void ax3500::ReadyState()
{
    static int token = 0;

    // Send cmd and process response
    if (ReadEncoders() == OK)
        UpdateOdometry();
    else {
       this->iErrorType=AX3500_ERROR_SERIALCOMM;
       SwitchToState(FAILURE_STATE);
    }

   	if (robot_data.bMotorsEnabled) {
        SetVWRef();
	}
    else       
        WriteMotorSpeed(0, 0);  // Set motor references to 0

    // No time critical functions called iteratively
    switch (token) {

        case 0:
            ReadBatteryVoltage(); // Send cmd and process response
        break;

        case 1:
            ReadAnalogInputs();   // Read analog inputs
        break;

        case 2:
            ReadTemperature();    // Read temperatures
        break;

        case 3:
            ReadDigitalInputs();  // Read digital inputs
        break;

        case 4: //WriteDigitalOutput( robot_data.digital_output_setvalue );
            ReadMotorControlMode();
        break;

        case 5:
            ReadInputControlMode();
        break;

		case 6:
            ReadAmps();
            token=-1;
        break;
			
        default:token=-1;
                break;
        }

	token++;

}

/*!	\fn void ax3500::FailureState()
 * 	\brief Actions in Failure State
*/
void ax3500::FailureState(){
	int timer = 1000000; //useconds
	static int recovery_cycles = 0;
	static int recovery_time = AX3500_DEFAULT_RECOVERY_TIME * threadData.dDesiredHz;

	// Setting the desired speed to 0
	SetSpeed(0.0, 0.0);
	
	recovery_cycles++;
	if(recovery_cycles >= recovery_time){ // Try to recover each second
		switch(iErrorType)	{

			ROS_ERROR("ax3500::FailureState: Trying to recover..");

			case AX3500_ERROR_OPENING: //Try to recover
				ROS_ERROR("ax3500::FailureState: Recovering from failure state (AX3500_ERROR_OPENING)");
				this->Close();
				usleep(timer);
				if (this->Open()==OK)
                    SwitchToState(INIT_STATE);
				break;

			case AX3500_ERROR_SERIALCOMM:
				ROS_ERROR("ax3500::FailureState: Recovering from failure state (AX3500_ERROR_SERIALCOMM)");
				this->Close();
				usleep(timer);
				if (this->Open()==OK){
                    char c;
                    int n = 0;
                    int j = 0;

                    while(j < 100000){	//Wait for confirmation
                        j++;
                        serial->ReadPort(&c, &n, 1);
                    }
                    SwitchToState(INIT_STATE);
				}

				break;

/*
			case AX3500_ERROR_TIMEOUT:
				log->AddError((char*)"ax3500::FailureState: Recovering from failure state (AX3500_ERROR_TIMEOUT.)");
				printf("ax3500::FailureState: Recovering from failure state (AX3500_ERROR_TIMEOUT.)\n");
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

/*!	\fn void ax3500::ToggleMotorPower(bool val)
 * 	\brief Switches on/off the motor
*/
void ax3500::ToggleMotorPower(bool val)
{
    this->robot_data.bMotorsEnabled = val;
    ROS_INFO("ax3500::ToggleMotorPower: Motor %s",val?"enabled":"disabled");
	//SwitchToState(READY_STATE);
}


/*!	\fn void ax3500::GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa)
	* Returns robot velocity and pose
*/
void ax3500::GetOdometry(double *vx, double *vy, double *va, double *px, double *py, double *pa)
{
    pthread_mutex_lock(&mutex_odometry);
        // Return values
        *vx = robot_pose.vx; *vy=robot_pose.vy; *va=robot_pose.va;
        *px=robot_pose.px; *py=robot_pose.py; *pa=robot_pose.pa;
    pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn pose ax3500::GetPose()
	* Returns the Pose (pos & vel)
*/
pose ax3500::GetPose(){
    return robot_pose;
}

/*!	\fn void ax3500::ResetOdometry()
	* Resets driver odometry
*/
void ax3500::ResetOdometry(){

	// Reset player local copy of the robot odometry
	pthread_mutex_lock(&mutex_odometry);
		robot_pose.px = 0;
		robot_pose.py = 0;
		robot_pose.pa = 0;
		robot_pose.vx = 0;
		robot_pose.vy = 0;
		robot_pose.va = 0;
	pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void ax3500::ModifyOdometry()
	* Set new odometry value 
*/
void ax3500::ModifyOdometry(  double px,  double py,  double pa )
{
	// Reset player local copy of the robot odometry
	pthread_mutex_lock(&mutex_odometry);
		robot_pose.px = px;
		robot_pose.py = py;
		robot_pose.pa = pa;
	pthread_mutex_unlock(&mutex_odometry);
}

/*!	\fn void ax3500::SetSpeedLimits()
	* Set robot linear and angular speed limits
	* linear speed in m/s
	* angular speed in rad/s
*/
void ax3500::SetSpeedLimits(float lin_speed, float ang_speed){

    robot_data.max_linear_speed = fabs(lin_speed);  // m/s
    robot_data.max_angular_speed = fabs(ang_speed); // rad/s

}

/*!	\fn void ax3500::SetSpeed(double lin_speed, double ang_speed)
	* Set robot's left and right track references from linear and angular values
	* \param lin_speed as double, desired linear speed
	* \param ang_speed as double, desired angular speed
*/
void ax3500::SetSpeed(double lin_speed, double ang_speed){
   
	pthread_mutex_lock(&mutex_odometry);
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

	pthread_mutex_unlock(&mutex_odometry);
	//ROS_ERROR("ax3500::SetSpeed: lin = %lf,  ang = %lf", robot_data.v_ref_mps, robot_data.w_ref_rads);
}

/*!	\fn void ax3500::GetVoltage()
	* Return the robot voltage value measured internally
*/
float ax3500::GetVoltage()
{
    return robot_data.voltage;
}

/*!	\fn int ax3500::GetEncoder(char enc)
	* Return the last value measured of encoder 'L'eft or 'R'ight
*/
int ax3500::GetEncoder(char enc)
{
    int encoder_value=0;
    switch (enc) {
        case 'L':
        case 'l':
            pthread_mutex_lock(&mutex_encoders);
                encoder_value = robot_data.encoder_left;
            pthread_mutex_unlock(&mutex_encoders);
            break;
        case 'R':
        case 'r':
            pthread_mutex_lock(&mutex_encoders);
                encoder_value = robot_data.encoder_right;
            pthread_mutex_unlock(&mutex_encoders);
            break;
        }

    return encoder_value;
}

/*!	\fn int ax3500::GetTemperature(int index)
	* Return the heat sink temperature measurement
*/
int ax3500::GetTemp(int index)
{
    if ((index<0)||(index>1)) return 0;
    else return robot_data.temperature[index];
}

/*!	\fn float ax3500::GetTemperature(int index)
	* Return the selected analog input measurement
*/
float ax3500::GetAnalogInput(int index)
{
    if ((index<0)||(index>1)) return 0;
    else return robot_data.analog_input[index];
}

/*!	\fn float ax3500::SetDigitalOutput
	* Public function for setting /resetting digital output
*/
void ax3500::SetDigitalOutput( bool value )
{
   robot_data.digital_output_setvalue = value;
}

/*!	\fn float ax3500::GetDigitalOutput
	* Public function for reading digital output
*/
bool ax3500::GetDigitalOutput()
{
   return robot_data.digital_output;
}

/*!	\fn bool ax3500::GetDigitalInput
	* Public function for reading digital inputs 0..2
*/
bool ax3500::GetDigitalInput(int input)
{
   if ((input>=0) && (input<=2))
      return robot_data.digital_input[input];
   else
      return false;
}

/*!	\fn float ax3500::CalculateBattery(double voltage)
	* Calculates the battery level using the discharge curve
*/
float ax3500::CalculateBattery(double voltage){
    // 53 V -> 100%
    // 50 V   -> 20%
    // 48.4 V -> 10%
    // 46 V   -> 0.1%
    double vLevels[4][2]={{53, 100}, {50, 20}, {48.4, 10}, {46, 0.1}};
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

/*!	\fn double ax3500::GetBatteryPercent()
	* Gets the percent of the total battery
*/
double ax3500::GetBatteryPercent(){
    return robot_data.battery;
}

/*!	\fn int ax3500::GetInputControlMode()
	* Returns the current input control mode (RC, RS232,  ..)
*/
int ax3500::GetInputControlMode(){
    return control_mode;
}

/*!	\fn const char * ax3500::GetInputControlModeString()
	*  Returns the current input control mode (RC, RS232,  ..)
*/
const char * ax3500::GetInputControlModeString(){
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

/*!	\fn int ax3500::GetInputControlMode()
	*  Returns the current Motor control mode (MIXED, SEPARATED, ..)
*/
int ax3500::GetMotorControlMode(){
    return motor_control_mode;
}

/*!	\fn const char * ax3500::GetInputControlModeString()
	*  Returns the current Motor control mode (MIXED, SEPARATED, ..)
*/
const char * ax3500::GetMotorControlModeString(){
	switch(motor_control_mode){
		case MCM_MIXED_OPEN:
			return "MIXED+OPEN";
		break;
		case MCM_MIXED_CLOSED:
			return "MIXED+CLOSED";
		break;
		case MCM_SEPARATED_OPEN:
			return "SEPARATED+OPEN";
		break;
		case MCM_SEPARATED_CLOSED:
            return "SEPARATED+CLOSED";
		break;
		default:
			return "UNKNOWN";
		break;
	}
}

/*!	\fn void ax3500::GetRPM(float *left_rpm, float *right_rpm)
	* Gets the rpm of each motor
*/
void ax3500::GetRPM(float *left_rpm, float *right_rpm){
    *left_rpm = robot_data.rpm_left;
    *right_rpm = robot_data.rpm_right;
}

/*!	\fn void ax3500::GetChannelReference(int *a, int *b)
	* Gets the channel's reference sent to the controller
*/
void ax3500::GetChannelReference(int *a, int *b){
    *a = robot_data.channel_A_ref;
    *b = robot_data.channel_B_ref;
}

/*!	\fn void ax3500::ConfigureConstants()
	* Configs the PID parameters
*/
void ax3500::ConfigureConstants(){

	// Constants to control the speed
	dWheelRpmToMps = (fDiameterWheel * Pi) / 60.0;          // conversion value from wheels rpm to mps = PI*Diameter/60
	dRpmToRef = (double) (AX3500_REF_FOR_MAX_RPM / MOTOR_MAX_RPM);     // conversion from MOTOR RPM to Reference        
	dRpmToWheelRpm = (double) (MOTOR_GEARBOX * dConversionFactor); 	// Conversion from MOTOR_RPM  to WHEEL RPM

	dWheelRpmToRpm = 1.0 / dRpmToWheelRpm;					// conversion from Wheel RPM to motor RPM
	dMpsToWheelRpm = 1.0 / dWheelRpmToMps;                    // conversion from MPS to wheels RPM
	dRefToRpm = 1.0 / dRpmToRef;                              // Conversion from Reference to MOTOR RPM

	dMpsToRef = dMpsToWheelRpm * dWheelRpmToRpm * dRpmToRef;        // Conversion from MPS to motor Reference
	dRefToMps = 1.0 / dMpsToRef;                                      // Conversion from Reference to MPS

	// PI parameters for velocity control. D parameter is not used
	pidMotor.KpL = (AX3500_MOTOR_DEF_MAX_REF_SPEED / 2.0)* pidMotor.KpL_divisor;        // P constant for linear velocity
	pidMotor.KpA = (AX3500_MOTOR_DEF_MAX_REF_SPEED / (Pi / 2.0))*pidMotor.KpA_divisor;  // P constant for angular velocity
	pidMotor.ErrSatIL = (MOTOR_MAX_RPM * dRpmToWheelRpm * dWheelRpmToMps) * pidMotor.ErrSatIL_divisor;                               // Max Linear Saturation Error: 10% of the max linear speed
	pidMotor.ErrSatIA = (2*(MOTOR_MAX_RPM * dRpmToWheelRpm * dWheelRpmToMps) / fDistanceBetweenWheels) * pidMotor.ErrSatIA_divisor;  // Max Angular Saturation Error: 10% of the max angular speed

	//ROS_ERROR("ax3500::ConfigureConstants: dWheelRpmToMps = %lf, dRpmToRef = %lf, dRpmToWheelRpm = %lf, dMpsToRef = %lf", dWheelRpmToMps, dRpmToRef, dRpmToWheelRpm, dMpsToRef);
}

/*!	\fn void ax3500::GetRobotSpeedLimits(float *lin_speed, float *ang_speed)
	* Gets the maximum configured speeds
*/
void ax3500::GetRobotSpeedLimits(float *lin_speed, float *ang_speed){
    *lin_speed = robot_data.max_linear_speed;
    *ang_speed = robot_data.max_angular_speed;
}

/*!	\fn void ax3500::SetMotorWheelParams(float diameter, float distance){
	* Sets the wheels' diameter and the distance between them
*/
void ax3500::SetMotorWheelParams(float diameter, float distance){
    if(diameter != 0.0 && distance != 0.0){
        fDiameterWheel = fabs(diameter);
        fDistanceBetweenWheels = fabs(distance);
        // Reconfigure PID parameters for the new values
        ConfigureConstants();
    }
}

/*!	\fn void ax3500::SetP(float p_linear_divisor, float p_angular_divisor)
	* Sets the P adjustable parameters
*/
void ax3500::SetP(float p_linear_divisor, float p_angular_divisor){
    pidMotor.KpL_divisor = p_linear_divisor;
    pidMotor.KpA_divisor = p_angular_divisor;
    // Reconfigures PID
    ConfigureConstants();
}

/*!	\fn void ax3500::SetI(float i_linear_constant, float i_angular_constant, float i_saturation_linear_error_divisor, float i_saturation_angular_error_divisor)
	* Sets the I adjustable parameters
*/
void ax3500::SetI(float i_linear_constant, float i_angular_constant, float i_saturation_linear_error_divisor, float i_saturation_angular_error_divisor){
    pidMotor.KiL = i_linear_constant;
    pidMotor.KiA = i_angular_constant;
    pidMotor.ErrSatIL_divisor = (double) i_saturation_linear_error_divisor;
    pidMotor.ErrSatIA_divisor = (double) i_saturation_angular_error_divisor;
    // Reconfigures PID
    ConfigureConstants();
}

/*!	\fn void ax3500::EnablePID(bool value)
	* Enables / Disables the PID controller
*/
void ax3500::EnablePID(bool value){
    bPID = value;
}

/*!	\fn void ax3500::EnablePID(bool value)
	* Sets the PID parametes
*/
void ax3500::SetPID(pid_params new_pid){
    pidMotor = new_pid;
    // Reconfigures PID
    ConfigureConstants();
}

/*!	\fn pid_params ax3500::GetPID()
	* Gets current PID params
*/
pid_params ax3500::GetPID(){
    return pidMotor;
}

/*! \fn ax3500_data ax3500::GetData()
 * 	\brief Gets all the data of the AX3500
*/
ax3500_data ax3500::GetData(){
    return robot_data;
}

/*! \fn bool ax3500::IsMotorEnabled()
 * 	\brief Return true if the motor is enabled
*/
bool ax3500::IsMotorEnabled(){
    return robot_data.bMotorsEnabled;
}

/*! \fn bool ax3500::IsPIDEnable()
 * 	\brief Return true if the PID is enabled
*/
bool ax3500::IsPIDEnable(){
    return bPID;
}

/*! \fn void ax3500::SetConversionFactor(double val)
 * 	\brief Sets the conversion factor (attribute dConversionFactor)
*/
void ax3500::SetConversionFactor(double val){
	this->dConversionFactor = val;
	// Reconfigures PID constants
    ConfigureConstants();
}

/*! \fn void ax3500::GetDesiredSpeed(float *linear, float *angular)
 * 	\brief Gets the desired linear and angular speed
*/
void ax3500::GetDesiredSpeed(float *linear, float *angular){
	*linear = (float) robot_data.v_ref_mps;
	*angular = (float) robot_data.w_ref_rads;
}

/*! \fn void ax3500::SetEncoderConfig(int config, int direction, int angular_dir)
 * 	\brief Sets the internal encoders' configuration
*/
void ax3500::SetEncoderConfig(int config, int direction, int angular_dir){
	this->iEncoderConfig = config;
	this->iEncoderDir = direction;
	this->iAngularSpeedDir = angular_dir;
	
    // Configure commands according to driver encoders mode
	if (this->encoders_mode==AX3500_ABSOLUTE_ENCODERS) {
	    if(this->iEncoderConfig == 1){
            sprintf(cmdEncLeft, "?q0\r"); // absolute encoders
            sprintf(cmdEncRight, "?q1\r");
	    }else{
            sprintf(cmdEncLeft, "?q1\r"); // absolute encoders
            sprintf(cmdEncRight, "?q0\r");
	    }
	}else {
	    if(this->iEncoderConfig == 1){
            sprintf(cmdEncLeft, "?q4\r"); // relative encoders
            sprintf(cmdEncRight, "?q5\r");
	    }else{
            sprintf(cmdEncLeft, "?q5\r"); // relative encoders
            sprintf(cmdEncRight, "?q4\r");
	    }
    }
}


/*!	\fn int ax3500::ReadAmps()
	* Sends the command to read the current applied to each motor
 	* \return OK if the read has been successful
	* \return ERROR if the read has failed
*/
int ax3500::ReadAmps(){
	int written_bytes=0;
	if(serial->WritePort((char*)"?A\r",&written_bytes, 3) != OK) {
		ROS_ERROR("ax3500::ReadDigitalInputs: Error sending message");
    }

	usleep(AX3500_SERIAL_DELAY);

	// read response from AX3500 - analog input
	bool bEnd=false;
	int i=0,j=0,n=0;
	char c;
    char buf[8];
	
	memset(buf,0,8);
	while (!( bEnd ) && j<50000) {	// wait until reading
		j++;
		serial->ReadPort(&c,&n, 1);
		if(n>0){ // some data received
			if (c!='\r') {
				buf[i]=c;
				i++;
				if (i>=6) {
				    bEnd=true;
					robot_data.current_consumption[0] = HexToDec(&buf[2],2);
					robot_data.current_consumption[1] = HexToDec(&buf[4],2);
                  //  ROS_INFO("ax3500::ReadAmps: read %s, a1 = %d, a2 = %d", buf, a1, a2);
                }
            }
        }
	}

	if(j==50000){
		ROS_ERROR("ax3500::ReadAmps: {%d} ErrData buf = %s",j,buf);
		this->err_counter++;
		return ERROR;
     }
	return OK;
}

/*!	\fn void ax3500::GetAmps(int *a1, int *a2)
	* Gets the motors consumption in Amps
 	* \return a1 as int, current consumed by motor 1
 	* \return a2 as int, current consumed by motor 2
*/
void ax3500::GetAmps(int *a1, int *a2){
	*a1 = this->robot_data.current_consumption[0];
	*a2 = this->robot_data.current_consumption[1];
}
