/*! \class Component
 *  \file Component.cc
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2009
 *  \brief Clase que definirá la estructura básica de todos los componentes
 * 	Define e implementa una serie de métodos y estructuras básicas que seguirán todos los componentes desarrollados por Robotnik
 * (C) 2009 Robotnik Automation, SLL
 *  All rights reserved.
 */

#include <pthread.h>

#include <unistd.h>
#include <stdio.h>
//#include <errno.h>
#include <string.h>
#include <stdlib.h>
//#include <stdint.h>
//#include <iostream>

#include  <guardian_node/Component.h>


/*! \fn void *AuxControlThread(void *threadParam)
 * \param threadParam as void *, parameters of thread
 * Function executing in the thread
*/
void *AuxControlThread(void *threadParam){
	Component *ComponentThread = (Component *)threadParam;
	ComponentThread->ControlThread();
    pthread_detach(pthread_self());
	return NULL;
}

/*! \fn Component::Component()
 *  \brief Constructor by default
 *	\param desired_hz as double, sets the desired frequency of the controlthread
*/
Component::Component(double desired_hz){
	// Set main flags to false
	bInitialized = bRunning = false;
	threadData.dRealHz = 0.0;
	if(desired_hz <= 0.0)
		desired_hz = DEFAULT_THREAD_DESIRED_HZ;
	threadData.dDesiredHz = desired_hz;
	iState = SHUTDOWN_STATE;
	// Realizar para cada una de las clases derivadas
	sComponentName.assign("Component");

	// mutex intitialization
	pthread_mutex_init(&mutexThread, NULL);
	// Real-Time parameters
	threadData.pthreadPar.prio = 25;				// Priority level 0[min]-80[max]
	threadData.pthreadPar.clock= CLOCK_REALTIME; 	// 0-CLOCK_MONOTONIC 1-CLOCK_REALTIME
}

/*! \fn Component::~Component()
 * Destructor by default
*/
Component::~Component(){

	pthread_mutex_destroy(&mutexThread);
}

/*! \fn ReturnValue Component::Setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
ReturnValue Component::Setup(){

	// Checks if has been initialized
	if(bInitialized){
		ROS_DEBUG("%s::Setup: Already initialized",sComponentName.c_str());
		return INITIALIZED;
	}

	//
	///////////////////////////////////////////////////
	// Setups another subcomponents if it's necessary //
	///////////////////////////////////////////////////

	/////////////////////////////////////////////////////
	// Allocates memory
	if(Allocate() != OK){
		ROS_ERROR("%s::Setup: Error in Allocate",sComponentName.c_str());
		return ERROR;
	}
	// Opens devices used by the component
	if(Open() != OK){
		ROS_ERROR("%s::Setup: Error in Open",sComponentName.c_str());
		return ERROR;
	}
	//
	// Configure the component
	if(Configure() != OK){
		ROS_ERROR("%s::Setup: Error in Configure",sComponentName.c_str());
		return ERROR;
	}

	bInitialized = true;

	return OK;
}

/*! \fn ReturnValue Component::ShutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
ReturnValue Component::ShutDown(){

	if(bRunning){
		ROS_ERROR("%s::ShutDown: Impossible while thread running, first must be stopped",sComponentName.c_str());
		return THREAD_RUNNING;
	}
	if(!bInitialized){
		ROS_ERROR("%s::ShutDown: Impossible because of it's not initialized", sComponentName.c_str());
		return NOT_INITIALIZED;
	}

	//
	///////////////////////////////////////////////////////
	// ShutDowns another subcomponents if it's necessary //
	///////////////////////////////////////////////////////

	if(Close() != OK){
		ROS_ERROR("%s::ShutDown: Error Closing", sComponentName.c_str());
		return ERROR;
	}
	if(Free() != OK){
		ROS_ERROR("%s::ShutDown: Error in Free", sComponentName.c_str());
		return ERROR;
	}

	bInitialized = false;

	return OK;
}

/*! \fn ReturnValue Component::Configure()
 * Configures devices and performance of the component
 * \return OK
 * \return ERROR if the configuration process fails
*/
ReturnValue Component::Configure(){

	return OK;
}

/*! \fn ReturnValue Component::Allocate()
 * Allocates virtual memory if it's necessary
 * \return OK
*/
ReturnValue Component::Allocate(){

	return OK;
}

/*! \fn ReturnValue Component::Free()
 * Frees allocated memory previously
 * \return OK
*/
ReturnValue Component::Free(){

	return OK;
}

/*! \fn ReturnValue Component::Open()
 * Opens used devices
 * \return OK
 * \return ERROR
*/
ReturnValue Component::Open(){

	return OK;
}

/*! \fn ReturnValue Component::Close()
 * Closes used devices
 * \return OK
 * \return ERROR
*/
ReturnValue Component::Close(){

	return OK;
}

/*! \fn ReturnValue Component::Start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
ReturnValue Component::Start(){
	pthread_attr_t attr;	// Thread attributed for the component threads spawned in this function

	// If the component is not initialized, we can't start
	if(!bInitialized){
		ROS_ERROR("%s::Start: the component is not initialized", sComponentName.c_str());
		return NOT_INITIALIZED;
	}
	//
	///////////////////////////////////////////////////
	// Starts another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	//
	if(!bRunning) {
		ROS_DEBUG("%s::Start: launching the thread", sComponentName.c_str());

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
		bRunning = true;
		// Creation of the main thread
		if(pthread_create(&threadData.pthreadId, &attr, AuxControlThread, this) != 0)	{
			ROS_ERROR("%s::Start: Could not create ControlThread", sComponentName.c_str());
			pthread_attr_destroy(&attr);
			bRunning = false;
			return ERROR;
		}

		pthread_attr_destroy(&attr);
		return OK;

	}else {
		ROS_DEBUG("%s::Start: the component's thread is already running", sComponentName.c_str());
		return THREAD_RUNNING;
	}

}

/*! \fn ReturnValue Component::Stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
ReturnValue Component::Stop(){

	if(!bRunning){
		ROS_ERROR( "%s::Stop: Thread not running", sComponentName.c_str());
		return THREAD_NOT_RUNNING;
	}
	//
	///////////////////////////////////////////////////
	// Stops another subcomponents, if it's necessary //
	///////////////////////////////////////////////////
	//
	ROS_DEBUG("%s::Stop: Stopping the thread", sComponentName.c_str());

	bRunning = false;

	usleep(100000);

	return OK;
}

/*!	\fn void Component::ControlThread()
 *	\brief All core component functionality is contained in this thread.
*/
void Component::ControlThread(){
	struct sched_param schedp;
	int policy = threadData.pthreadPar.prio? SCHED_FIFO : SCHED_OTHER;
	struct timespec now, next, interval;
	struct timespec last_time;  // Necesarry to calculates real frec
	long diff;
	unsigned long sampling_period_us;

	// Robotnik thread priority and timing management
	memset(&schedp, 0, sizeof(schedp));
	schedp.sched_priority = threadData.pthreadPar.prio;
	sched_setscheduler(0, policy, &schedp);
	clock_gettime(threadData.pthreadPar.clock, &now);
 	last_time = next = now;
	next.tv_sec++;  // start in next second?
	sampling_period_us =  (long unsigned int) (1.0 / threadData.dDesiredHz * 1000000.0);
	interval.tv_sec = sampling_period_us / USEC_PER_SEC;  // interval parameter in uS
	interval.tv_nsec = (sampling_period_us % USEC_PER_SEC)*1000;

	ROS_DEBUG("%s::ControlThread: Starting the thread", sComponentName.c_str());
	// Begin thread execution on init state
	SwitchToState(INIT_STATE);

	while(bRunning) { // Executes state machine code while bRunning flag is active
		clock_nanosleep(threadData.pthreadPar.clock, TIMER_ABSTIME, &next, NULL);
		clock_gettime(threadData.pthreadPar.clock, &now);
		next.tv_sec += interval.tv_sec;
		next.tv_nsec += interval.tv_nsec;
		tsnorm(&next);

		diff = calcdiff(now, last_time);
		last_time = now;
		// Thread frequency update
		threadData.dRealHz = 1.0/(diff / 1000000.0); // Compute the update rate of this thread

		switch (iState){
			case INIT_STATE:
				InitState();
			break;
			case STANDBY_STATE:
				StandbyState();
			break;
			case READY_STATE:
				ReadyState();
			break;
			case EMERGENCY_STATE:
				EmergencyState();
			break;
			case FAILURE_STATE:
				FailureState();
			break;
			default:
			break;
		}
		AllState();
	}
	SwitchToState(SHUTDOWN_STATE);
	usleep(100000);	// Sleep for 100 milliseconds and then exit

}

/*!	\fn void Component::InitState()
 *	\brief Actions performed on initial state
*/
void Component::InitState(){

}

/*!	\fn void Component::StandbyState()
 *	\brief Actions performed on Standby state
*/
void Component::StandbyState(){

}

/*!	\fn void Component::ReadyState()
 *	\brief Actions performed on ready state
*/
void Component::ReadyState(){

}

/*!	\fn void Component::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void Component::EmergencyState(){

}

/*!	\fn void Component::FailureState()
 *	\brief Actions performed on failure state
*/
void Component::FailureState(){

}

/*!	\fn void Component::AllState()
 *	\brief Actions performed on all states
*/
void Component::AllState(){

}

/*!	\fn double Component::GetUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double Component::GetUpdateRate(){
	return threadData.dRealHz;
}

/*!	\fn States Component::GetState()
 * 	\brief returns the state of the component
*/
States Component::GetState(){
	return iState;
}

/*!	\fn char *Component::GetStateString()
 *	\brief Gets the state of the component as string
*/
char *Component::GetStateString(){
	switch(iState){
		case INIT_STATE:
			return (char *)"INIT     ";
		break;
		case STANDBY_STATE:
			return (char *)"STANDBY  ";
		break;
		case READY_STATE:
			return (char *)"READY    ";
		break;
		case EMERGENCY_STATE:
			return (char *)"EMERGENCY";
		break;
		case FAILURE_STATE:
			return (char *)"FAILURE  ";
		break;
		case SHUTDOWN_STATE:
			return (char *)"SHUTDOWN  ";
		break;
		default:
			return (char *)"UNKNOWN  ";
		break;
	}
}

/*!	\fn void Component::SwitchToState(States new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void Component::SwitchToState(States new_state){

	if(new_state == iState)
		return;

	iState = new_state;
	switch (new_state) {
		case INIT_STATE:
			ROS_DEBUG("%s:: switching to INIT state", sComponentName.c_str());		
			break;
		case STANDBY_STATE:
			ROS_DEBUG( "%s:: switching to STANDBY state", sComponentName.c_str());			
			break;
		case READY_STATE:
			ROS_DEBUG( "%s:: switching to READY state", sComponentName.c_str());			
			break;
		case EMERGENCY_STATE:
			ROS_DEBUG("%s:: switching to EMERGENCY state", sComponentName.c_str());		
			break;
		case FAILURE_STATE:
			ROS_DEBUG("%s:: switching to FAILURE state", sComponentName.c_str());
			break;
        case SHUTDOWN_STATE:
			ROS_DEBUG("%s:: switching to SHUTDOWN state", sComponentName.c_str());
			break;
		default:
			ROS_DEBUG("%s:: switching to UNKNOWN state", sComponentName.c_str());
			break;
	}
}

/*!	\fn void Component::SetLogParameters(int port, char *ip_dest)
 * 	Sets the ip and port dest of the remote rlcLog
 * 	\param port as an integer, new port for the communication with rlcLog server
 *  \param ip_dest as char *, new IP for the communication with rlcLog server
*/
void Component::SetLogParameters(int port, char *ip_dest){
	iLogPort = port;
	sLogIp.assign(ip_dest);
}

/*!	\fn void Component::SetLogState(bool log)
 * 	Sets the remotelog for logging or not
 * 	\param log as bool, if true, enables the rlcLog, else disables the rlcLog
*/
void Component::SetLogState(bool log){

}

/*!	\fn ReturnValue Component::Wait(u_seconds)
 * 	Waits for the end of all the created threads
 *  \param u_seconds as unsigned int, max time waiting for the threads
 * 	\return NOT_RUNNING if there is no threads running
 *  \return OK if waits successfully
 *  \return ERROR if an error is produced
*/
//ReturnValue Component::Wait(unsigned int u_seconds){
    // UNUSED
//	return OK;
//}

/*!	\fn ReturnValue Component::GetThreadData(thread_data *data)
 * 	Gets the data of the current thread
 *  \return OK if the id is OK
 *  \return ERROR if the id doesn't exist
*/
ReturnValue Component::GetThreadData(thread_data *data){
    pthread_t auxId = pthread_self();
    ReturnValue ret = ERROR;
    // Main thread
    if(auxId == threadData.pthreadId){
        *data = threadData;
        printf("Component::GetThreadData: Main thread");
        return OK;
    }
    // Search the Id in the list
    pthread_mutex_lock(&mutexThread);
        for(unsigned int i = 0; i < vThreadData.size(); i++){
            if(auxId == vThreadData[i].pthreadId){  // Encontrado
                *data = vThreadData[i];
                ret = OK;
                break;
            }
        }
    pthread_mutex_unlock(&mutexThread);

    return ret;
}

/*!	\fn ReturnValue Component::CreateTask(int prio, double frec, void *(*start_routine)(void*))
 * 	Creates new thread executing specific tasks
 *  \param prio as int, Thread priority level ( 0[min]-80[max] )
 *  \param frec as double, thread frequency
 *  \param start_routine as void*(*)(void*), function wich will execute the task
 * 	\return THREAD_NOT_RUNNING if the component is not running
 *  \return OK if waits successfully
 *  \return ERROR if an error is produced creating the thread
*/
ReturnValue Component::CreateTask(int prio, double frec, void *(*start_routine)(void*)){
	pthread_attr_t attr;	// Thread attributed for the component threads spawned in this function
    thread_data new_thread;
    if(frec <= 0.0)
        frec = DEFAULT_THREAD_DESIRED_HZ;
    if(prio < 0)
        prio = DEFAULT_THREAD_PRIORITY;
    //
	// If the component is not initialized, we can't start
	if(!bRunning){
		ROS_ERROR("%s::CreateTask: the component is not running", sComponentName.c_str());
		return THREAD_NOT_RUNNING;
	}

    ROS_DEBUG("%s::CreateTask: launching the thread with priority = %d and frequency = %lf", sComponentName.c_str(), prio, frec);

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    // Protect the creation and modification of threads with a mutex
    pthread_mutex_lock(&mutexThread);

        // Creation of the main thread
        if(pthread_create(&new_thread.pthreadId, &attr, start_routine, this) != 0)	{
            ROS_ERROR("%s::CreateTask: Could not create new thread", sComponentName.c_str());
            pthread_attr_destroy(&attr);
            pthread_mutex_unlock(&mutexThread);
            return ERROR;
        }
        pthread_attr_destroy(&attr);
        new_thread.dDesiredHz = frec;
        new_thread.dRealHz = 0.0;
        new_thread.pthreadPar.prio = prio;
        new_thread.pthreadPar.clock = CLOCK_REALTIME;
        // Adds information for the new thread
        vThreadData.push_back(new_thread);
        ROS_DEBUG("%s::CreateTask: New thread created (%d, %d, %lf) at %d position", sComponentName.c_str(),(int) new_thread.pthreadId, new_thread.pthreadPar.prio, new_thread.dDesiredHz, vThreadData.size());

    pthread_mutex_unlock(&mutexThread);

    return OK;
}

/*!	\fn int Component::Clock()
 * 	Returns clock REAL_TIME or MONOTONIC
 *  to allow derived classes to measure time, calls to clock_gettime
 */
int Component::GetClock()
{
    return threadData.pthreadPar.clock;
}
