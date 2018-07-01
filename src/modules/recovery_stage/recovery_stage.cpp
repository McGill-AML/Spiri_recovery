/*
 * @file recovery_stage.cpp
 * 
 * takes sensor data as input and computes the recovery stage, passed to mc_att_control
 *
 * Stage 1: Point away from the wall based on impact characterization
 * Stage 2: Go to hover
 * Stage 0: Normal flight 
 *
 * @author Gareth Dicker<dicker.gareth@gmail.com>
 *
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/control_state.h>

#include <uORB/topics/impact_detection.h>
#include <uORB/topics/impact_characterization.h>
#include <uORB/topics/impact_recovery_stage.h>
#include <uORB/topics/recovery_control.h>
#include <uORB/topics/vehicle_local_position.h>


#include <uORB/topics/parameter_update.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */

extern "C" __EXPORT int recovery_stage_main(int argc, char *argv[]);

int recovery_stage_thread_main(int argc, char *argv[]);

static void usage(const char *reason){
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: recovery_stage {start|stop|status} [-p <additional params>]\n\n");
}

int recovery_stage_main(int argc, char *argv[]){
	if (argc < 2) {
		PX4_INFO("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			PX4_INFO("impact detection already running\n");
			/* this is not an error */
			return 0;
		}
		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("recovery_stage",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 recovery_stage_thread_main,
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

int recovery_stage_thread_main(int argc, char *argv[])
{

	PX4_INFO("recovery_stage starting\n");

	thread_running = true;

	bool ATTITUDE_STABLE = false;
	bool BODY_RATES_STABLE = false;
	bool VERTICAL_VELOCITY_STABLE = false;
	bool STABLE_SAMPLE = false;

	int countSamples = 0;
	const int sampleThresh = 2;
	const double quatErrThresh = 0.15;
	const double rollPitchThresh = 0.2;
	const double verticalVelocityThresh = 0.5; 
	const double bodyRatesThresh = 2.0; 	// play with this

	// set up subscribers
	int _sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	int _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	int _detection_sub = orb_subscribe(ORB_ID(impact_detection));
	int _characterization_sub = orb_subscribe(ORB_ID(impact_characterization));
	int _recovery_control_sub = orb_subscribe(ORB_ID(recovery_control));
	int _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));			/**< vehicle local position */
	
	// declare local copies
	struct sensor_accel_s            		_sensor_accel;
	struct control_state_s					  _ctrl_state;
	struct impact_detection_s				   _detection;
	struct impact_characterization_s 	_characterization;
	struct impact_recovery_stage_s		  _recovery_stage;
	struct recovery_control_s           _recovery_control;	
	struct vehicle_local_position_s		       _local_pos;

	// set them to zero initially
	memset(&_sensor_accel, 0, sizeof(_sensor_accel));	
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_detection, 0, sizeof(_detection));
	memset(&_characterization, 0, sizeof(_characterization));
	memset(&_recovery_stage, 0, sizeof(_recovery_stage));
	memset(&_recovery_control, 0, sizeof(_recovery_control));
	memset(&_local_pos, 0, sizeof(_local_pos));	

	// declare update flags
	bool updated_sensor_accel;
	bool updated_ctrl_state;
	bool updated_detection;
	bool updated_characterization;
	bool updated_recovery_control;	
	bool updated_local_position;

	orb_advert_t _recovery_stage_pub = orb_advertise(ORB_ID(impact_recovery_stage), &_recovery_stage);

	struct pollfd fds[1];
	fds[0].fd = _sensor_accel_sub;
	fds[0].events = POLLIN;

    while(!thread_should_exit){  

		int ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		if (ret == 0) {
			continue;
		}

		if (fds[0].revents & POLLIN) {
	    	// poll for subscription updates
			orb_check(_sensor_accel_sub, &updated_sensor_accel);
			orb_check(_ctrl_state_sub, &updated_ctrl_state);
			orb_check(_detection_sub, &updated_detection);
			orb_check(_characterization_sub, &updated_characterization);
			orb_check(_recovery_control_sub, &updated_recovery_control);
			orb_check(_local_pos_sub, &updated_local_position);	

			// make local copies of uORB topics
			if(updated_sensor_accel){
				orb_copy(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
			}
			if(updated_ctrl_state){
				orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
			}
			if(updated_detection){
				orb_copy(ORB_ID(impact_detection), _detection_sub, &_detection);
			}
			if(updated_characterization){
				orb_copy(ORB_ID(impact_characterization), _characterization_sub, &_characterization);
			}
			if(updated_recovery_control){		
				orb_copy(ORB_ID(recovery_control), _recovery_control_sub, &_recovery_control);		
			}
			if(updated_local_position){
				orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
			}

			if(_characterization.accelRefIsComputed && _recovery_stage.recoveryIsReset == false){

				math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
				math::Vector<3> angles = q_att.to_euler();


				float roll = angles(0);
				float pitch = angles(1);
				float quatErrorX = _recovery_control.quatError[1];
				float quatErrorY = _recovery_control.quatError[2];
				float rollRate = _ctrl_state.roll_rate;
				float pitchRate = _ctrl_state.pitch_rate;
				float yawRate = _ctrl_state.yaw_rate;
				float verticalVelocity = _local_pos.vz;

		        if (_recovery_stage.recoveryStage == 1){

		        	// instead check the normal, 
					ATTITUDE_STABLE = fabs(quatErrorX) < quatErrThresh && fabs(quatErrorY) < quatErrThresh;
					BODY_RATES_STABLE = fabs(rollRate) < bodyRatesThresh && fabs(pitchRate) < bodyRatesThresh;
		        	STABLE_SAMPLE = ATTITUDE_STABLE && BODY_RATES_STABLE;

					if(STABLE_SAMPLE){
						countSamples += 1; 
					}
					if(countSamples >= sampleThresh){
		            	_recovery_stage.recoveryStage = 2;
						countSamples = 0;
					}	            		
		        }
		        else if(_recovery_stage.recoveryStage == 2){

					ATTITUDE_STABLE = fabs(roll) < rollPitchThresh && fabs(pitch) < rollPitchThresh;
					BODY_RATES_STABLE = fabs(rollRate) < bodyRatesThresh && fabs(pitchRate) < bodyRatesThresh && fabs(yawRate) < bodyRatesThresh;
					VERTICAL_VELOCITY_STABLE = fabs(verticalVelocity) < verticalVelocityThresh;

					//modified to not use VERTICAL_VELOCITY_STABLE
		        	STABLE_SAMPLE = ATTITUDE_STABLE && BODY_RATES_STABLE && VERTICAL_VELOCITY_STABLE;

					if(STABLE_SAMPLE){
						countSamples += 1; 
					}
					if(countSamples >= sampleThresh){
		            	_recovery_stage.recoveryStage = 0;
		        		_recovery_stage.recoveryIsReset = true;
						countSamples = 0;
					}	
		        }
		        else{
					_recovery_stage.recoveryStage = 1; // executes upon opening this loop
		        }
		    }
			
			if (_characterization.accelRefIsComputed == false && _detection.inRecovery == false && _recovery_stage.recoveryIsReset == true){
				_recovery_stage.recoveryIsReset = false;
			}

        	orb_publish(ORB_ID(impact_recovery_stage), _recovery_stage_pub, &_recovery_stage);
    	}
    }

	PX4_INFO("recovery_stage exiting.\n");
	thread_running = false;

    return OK;
}




