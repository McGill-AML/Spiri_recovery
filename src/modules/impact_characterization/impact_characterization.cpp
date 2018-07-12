/**
 * @file impact_characterization.cpp
 * 
 * Calculates accelReference for recovery controller
 * 
 * modules recovery_stage and mc_att_control make use of this flag.
 *
 * @author Gareth Dicker<dicker.gareth@gmail.com>
 * @author2 Fiona Chui<fiona.m.chui@gmail.com>
 */
#include <modules/impact_characterization/impact_characterization.h> 


// CLASS FUNCTIONS

extern "C" __EXPORT int impact_characterization_main(int argc, char *argv[]);


 QuaternionQueue::QuaternionQueue()
{
	head = tail = 0;
}

void QuaternionQueue::quatqueue(math::Quaternion quat){
	if (tail + 1 == head || ((tail + 1 == QUATQUEUESIZE) && !head)){
		//queue is full
		head ++;
		if (head == QUATQUEUESIZE) head = 0;
	}
	tail ++;
	if (tail == QUATQUEUESIZE) tail = 0;
	queue[tail] = quat;
	return;
}

math::Quaternion QuaternionQueue::readquatqueue(int queueidx){
	if (queueidx >= QUATQUEUESIZE){
		PX4_INFO("ERROR: requested to access quat queue out of range");
		math::Quaternion tempquat(1,0,0,0);
		return tempquat;
	}
	else{
		if (head+queueidx >= QUATQUEUESIZE) return queue[head+queueidx-QUATQUEUESIZE];
		else return queue[head+queueidx];
	}

}

// CUSTOM FUNCTIONS
math::Vector<3> crossProduct(const math::Vector<3> v1, const math::Vector<3> v2){
	/**
	 * cross product (added by Fiona)
	 */
 	math::Vector<3> res;
	res.data[0] = v1.data[1]*v2.data[2] - v1.data[2]*v2.data[1];
	res.data[1] = v1.data[2]*v2.data[0] - v1.data[0]*v2.data[2];
	res.data[2] = v1.data[0]*v2.data[1] - v1.data[1]*v2.data[0];
	return res;

}

float signf(const float number){
	if (number < 0.0f) return -1.0f;
	else return 1.0f;
}

float rad2deg(const float angle){
	return angle*180.0f/(float)M_PI;
}

// DECLARE CONSTANTS

const math::Vector<3> inertialFrameGravityDirection(0.0f,0.0f,1.0f); //Navi's inertial frame is NED
const math::Vector<3> inertialFrameNegGravityDirection(0.0f,0.0f,-1.0f);
const math::Vector<3> bodyFrameGravityDirection(0.0f,0.0f,1.0f); //Navi's body frame is also NED 
const math::Vector<3> bodyFrameNegGravityDirection(0.0f,0.0f,-1.0f); 


//DECLARE STATICS
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

//MAIN FUNCTIONS
extern "C" __EXPORT int impact_characterization_main(int argc, char *argv[]);

static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: impact_characterization {start|stop|status} [-p <additional params>]\n\n");
}

int impact_characterization_main(int argc, char *argv[]){
		if (argc < 2) {
		PX4_INFO("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("impact_characterization already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("impact_characterization",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 impact_characterization_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("\trunning\n");

		} else {
			PX4_INFO("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}


int impact_characterization_thread_main(int argc, char *argv[])
{

	PX4_INFO("impact_characterization starting\n");

	thread_running = true;
	///////////////////// SUBSCRIPTIONS//////////////////////////
	// set up subscribers
	int _sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	int _sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	//int _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));

	//Custom:_v_att instead of _ctrl_state	
	int _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	int _detection_sub = orb_subscribe(ORB_ID(impact_detection));
	int _recovery_stage_sub = orb_subscribe(ORB_ID(impact_recovery_stage));

	// declare local copies
	struct sensor_accel_s             	  _sensor_accel;
	struct sensor_gyro_s              _sensor_gyro;
	//struct control_state_s			  _ctrl_state;
	//Custom:_v_att instead of _ctrl_state
	struct vehicle_attitude_s		_v_att;

	struct impact_detection_s 				 _detection;
	struct impact_recovery_stage_s  	_recovery_stage;

	// struct debug_s								 _debug;

	// set them to zero initially
	memset(&_sensor_accel, 0, sizeof(_sensor_accel));	
	memset(&_sensor_gyro, 0, sizeof(_sensor_gyro));
	//Custom:_v_att instead of _ctrl_state
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_detection, 0, sizeof(_detection));
	memset(&_recovery_stage, 0, sizeof(_recovery_stage));

	// declare update flags
	bool updated_sensor_accel;
	bool updated_sensor_gyro;
	//Custom: _v_att instead of _ctrl_state
	bool updated_v_att;
	bool updated_detection;
	bool updated_recovery_stage;
	////////////////////////////////////////////////////////////

	////////////////////////PUBLICATIONS////////////////////////
	struct impact_characterization_s   _characterization;
	memset(&_characterization, 0, sizeof(_characterization));
	orb_advert_t _characterization_pub = orb_advertise(ORB_ID(impact_characterization), &_characterization);
	////////////////////////////////////////////////////////////

	///////////////////////////POLLING///////////////////////////
	struct pollfd fds[1];
	fds[0].fd = _sensor_accel_sub;
	fds[0].events = POLLIN;
    ////////////////////////////////////////////////////////////

    ///////////////////////LOCAL VARIABLES//////////////////////
	int cyclesAfterImpactDetected = 0; 
	//int fuzzyInputCalculated_array[4] = {0,0,0,0};

	QuaternionQueue q_att_log;
	math::Quaternion quat_preImpact;
	math::Matrix<3, 3> R_preImpact;

	math::Vector<3> wallNormal_vect(0.0f, 0.0f, 0.0f);
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////

                
    while(!thread_should_exit){  
    	//TODO: check if armed

		int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		if (ret == 0) {
			continue;
		}

		if (fds[0].revents & POLLIN){		
		   	// poll for subscription updates

			orb_check(_sensor_accel_sub, &updated_sensor_accel);
			orb_check(_sensor_gyro_sub, &updated_sensor_gyro);
			//orb_check(_ctrl_state_sub, &updated_ctrl_state);
			//Custom: _v_att instead of _ctrl_state
			orb_check(_v_att_sub, &updated_v_att);
			orb_check(_detection_sub, &updated_detection);
			orb_check(_recovery_stage_sub, &updated_recovery_stage);

			// make local copies of uORB topics
			if(updated_sensor_accel){
				orb_copy(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
			}
			if(updated_sensor_gyro){
				orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub, &_sensor_gyro);
			}			
			/*if(updated_ctrl_state){
				orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
			}*/
			if (updated_v_att) {
				orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
			}
			if(updated_detection){
				orb_copy(ORB_ID(impact_detection), _detection_sub, &_detection);
			}
			if(updated_recovery_stage){
				orb_copy(ORB_ID(impact_recovery_stage), _recovery_stage_sub, &_recovery_stage);
			}

			//Custom: _v_att instead of _ctrl_state
			math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);


			if (_detection.inRecovery){
				if(!_characterization.accelRefIsComputed){					
					if (cyclesAfterImpactDetected == 0){
						// calculate pre-impact rotation matrix							
						quat_preImpact = q_att_log.readquatqueue(QUATQUEUESIZE-PREIMPACTCYCLES-1);
						R_preImpact = quat_preImpact.to_dcm();
						
						// calculate wallNormalDirection --> may move this to another time step
						math::Matrix<3, 3> R = q_att.to_dcm();
						math::Vector<3> accel_in_gs(_sensor_accel.x/9.81f, _sensor_accel.y/9.81f, _sensor_accel.z/9.81f);
						math::Vector<3> inertialAcceleration = R*accel_in_gs + inertialFrameGravityDirection;
						math::Vector<3> wallNormalDirectionWorld(inertialAcceleration(0),inertialAcceleration(1),0.0f);
						wallNormal_vect = wallNormalDirectionWorld.normalized();
						_characterization.wallNormal[0] = wallNormal_vect(0);
						_characterization.wallNormal[1] = wallNormal_vect(1);
						_characterization.wallNormal[2] = wallNormal_vect(2);
					}	

					math::Vector<3> accelReference_vect = wallNormal_vect*(-0.75f*9.81f);
					// math::Vector<3> accelReference_vect = wallNormal_vect*(9.81f*_characterization.fuzzyOutput);
					
					_characterization.accelReference[0] = accelReference_vect(0);
					_characterization.accelReference[1] = accelReference_vect(1);
					_characterization.accelReference[2] = accelReference_vect(2);
					_characterization.accelRefIsComputed = true;

					cyclesAfterImpactDetected ++;
					
				} // end if(!accelRefCalculated)	

			} //end if(_impact.impactIsDetected)

			//reset condition
			if (_characterization.accelRefIsComputed && _recovery_stage.recoveryIsReset){
				cyclesAfterImpactDetected = 0;


				/*fuzzyInputCalculated_array[0] = 0;
				fuzzyInputCalculated_array[1] = 0;
				fuzzyInputCalculated_array[2] = 0;
				fuzzyInputCalculated_array[3] = 0;*/

				wallNormal_vect(0) = 0.0f;
				wallNormal_vect(1) = 0.0f;
				wallNormal_vect(2) = 0.0f;

			    /*_characterization.fuzzyInput[0] = 0.0f;
			    _characterization.fuzzyInput[1] = 0.0f;
			    _characterization.fuzzyInput[2] = 0.0f;
			    _characterization.fuzzyInput[3] = 0.0f;
				_characterization.fuzzyOutput = 0.0f;*/		
			    _characterization.wallNormal[0] = 0.0f;
			    _characterization.wallNormal[1] = 0.0f;
			    _characterization.wallNormal[2] = 0.0f;
			    _characterization.accelReference[0] = 0.0f; 
			    _characterization.accelReference[1] = 0.0f;
			    _characterization.accelReference[2] = 0.0f;
				_characterization.accelRefIsComputed = false;	
			}
			q_att_log.quatqueue(q_att); //record attitude history

        	orb_publish(ORB_ID(impact_characterization), _characterization_pub, &_characterization);
			 
    	} //end polling

    } //end while

	PX4_INFO("impact_characterization exiting.\n");

	thread_running = false;

    return OK;
} //end main



