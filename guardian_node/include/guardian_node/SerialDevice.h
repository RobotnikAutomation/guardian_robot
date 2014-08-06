/** \file SerialDevice.h
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2010
 *
 * \brief SerialDevice Class
 * (C) 2010 Robotnik Automation, SLL
 * Class to manage a serial port connection
*/

#ifndef __SERIALDEV_H
	#define __SERIALDEV_H

#include <stdio.h>
#include <guardian_node/Component.h>

#define BUFFER						512
#define DEFAULT_PORT 				"/dev/ttyS1"
#define DEFAULT_TRANSFERRATE 		19200
#define DEFAULT_PARITY 				"odd" //"even" "odd" "none"
#define DEFAULT_SIZE_ARRAY			128
#define DEFAULT_DATA_SIZE			7

class SerialDevice: public Component{

private:
	//! File descriptor
	int fd;
	//!
	//unsigned char SendBuf[BUFFER];
	//!
	//unsigned char RecBuf[BUFFER];
protected:
	//! Device's name
	char cDevice[DEFAULT_SIZE_ARRAY];
	//! Parity for input and output: EVEN, ODD, NONE
	char cParity[DEFAULT_SIZE_ARRAY];
	//! BaudRate: 9600, 19200, 38400, 115200
	int iBaudRate;
	//! Character size mask. Values are CS5, CS6, CS7, or CS8.
	int iBitDataSize;

protected:
	//! Mode to open the device
	int iOpenMode;
	//! Entrada canónica-> La entrada canónica es orientada a línea. Los caracteres se meten en un buffer hasta recibir un CR o LF.
	bool bCanon;

public:
	//! Public Constructor
	SerialDevice(double hz);
	//! Public Constructor
	SerialDevice(const char *device, int baudrate,const char *parity, int datasize, double hz);
	//! Public Destructor
	~SerialDevice(void);
	//! Sends data to the serial port
	ReturnValue WritePort(char *chars, int *written_bytes, int length);
	//! Reads the serial port
	ReturnValue ReadPort(char *result, int *read_bytes, int num_bytes);
	//! Clean port buffer
	ReturnValue Flush();
	//! Returns opened device
	char *GetDevice();
	//! Sets the Canonical input. False by default
	void SetCanonicalInput(bool value);
    //! Sets open mode
    void SetOpenMode(int mode);
protected:
    //! Opens used devices
    //! @return OK
    //! @return ERROR
    virtual ReturnValue Open();
    //! Closes used devices
    //! @return OK
    //! @return ERROR
    virtual ReturnValue Close();
	//! Set serial communication speed.
    //! Valid values: 9600, 19200, 38400, 115200
    //! @return SERIALDEV_OK
    //! @return SERIALDEV_ERROR
	virtual ReturnValue Configure(void);
	//!	Set serial communication speed.
	ReturnValue SetTermSpeed(int speed);

};

#endif
