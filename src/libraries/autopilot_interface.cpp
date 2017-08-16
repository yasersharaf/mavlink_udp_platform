/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
// Autopilot_Interface::
// Autopilot_Interface(Serial_Port *serial_port_)

Autopilot_Interface::Autopilot_Interface(Client* client_){
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	client = client_; // client port management object
}

Autopilot_Interface::Autopilot_Interface(UDP_Client* udp_client_){
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	udp_client = udp_client_; // client port management object
}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = udp_client->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;
//			printf("id: %d\n",message.msgid);
			// Handle Message ID
//			switch (message.msgid)
//			{

//				case MAVLINK_MSG_ID_HEARTBEAT:
//				{
//					printf("MAVLINK_MSG_ID_HEARTBEAT\n");
//					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
//					current_messages.time_stamps.heartbeat = get_time_usec();
//					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
//					break;
//				}
//				case 27:
//				{}
//				case MAVLINK_MSG_ID_STATUSTEXT:
//				{
//					printf("STATUSTEXT\n");
//					mavlink_msg_statustext_decode(&message, &(current_messages.statustext));
////					current_messages.time_stamps.heartbeat = get_time_usec();
//					printf(current_messages.statustext.text);
//					printf("\n Severity: %d\n\n", current_messages.statustext.severity);
//					break;
//				}
//
//				case MAVLINK_MSG_ID_SYS_STATUS:
//				{
//					printf("MAVLINK_MSG_ID_SYS_STATUS\n");
//					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
//					current_messages.time_stamps.sys_status = get_time_usec();
//					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_BATTERY_STATUS:
//				{
//					printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
//					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
//					current_messages.time_stamps.battery_status = get_time_usec();
//					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_RADIO_STATUS:
//				{
//					printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
//					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
//					current_messages.time_stamps.radio_status = get_time_usec();
//					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
//				{
//					printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED RECEIVED \n");
//					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
//					current_messages.time_stamps.local_position_ned = get_time_usec();
//					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
//				{
//					printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
//					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
//					current_messages.time_stamps.global_position_int = get_time_usec();
//					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
//				{
//					printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
//					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
//					current_messages.time_stamps.position_target_local_ned = get_time_usec();
//					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
//				{
//					printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
//					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
//					current_messages.time_stamps.position_target_global_int = get_time_usec();
//					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_HIGHRES_IMU:
//				{
//					printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
//					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
//					current_messages.time_stamps.highres_imu = get_time_usec();
//					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
//					break;
//				}
//
//				case MAVLINK_MSG_ID_ATTITUDE:
//				{
//					printf("MAVLINK_MSG_ID_ATTITUDE\n");
//					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
//					current_messages.time_stamps.attitude = get_time_usec();
//					this_timestamps.attitude = current_messages.time_stamps.attitude;
//					break;
//				}
//
//				default:
//				{
////					 printf("Warning, did not handle message id %i\n",message.msgid);
//					break;
//				}
//
//
//			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(1000); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	int len;
	// do the write
//	if (current_gps.lat !=0 && current_gps.lon !=0){
	if (1){
		len = udp_client->write_message(message);
//		printf("#%d",write_count);
	}
	else{
		len = 1;
	}


	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	else if(len==1)
		printf("\nCouldn't write\n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


void
Autopilot_Interface::
write_gps_raw_int()
{

       mavlink_gps_raw_int_t sp = current_gps;


//       printf("lat= %d, lon= %d, alt = %d\n",current_gps.lat,current_gps.lon,current_gps.alt);
       // --------------------------------------------------------------------------
       //   ENCODE
       // --------------------------------------------------------------------------

       mavlink_message_t message;

       mavlink_msg_gps_raw_int_encode(system_id, companion_id, &message, &sp);
//       printf("GPS ENCODED!\n");
       // --------------------------------------------------------------------------
       //   WRITE
       // --------------------------------------------------------------------------

       // do the write
       int len = write_message(message);

       // check the write
       if ( len <= 0 )
               fprintf(stderr,"WARNING: could not send GPS RAW INT \n");
       //      else
       //              printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_ta

       return;
}


void
Autopilot_Interface::
write_mocap_floats()
{

	current_mocap_value.usec = get_time_usec();

	__mavlink_vicon_position_estimate_t pos_est = current_mocap_value;
//	__mavlink_sim_state_t allposes = Mocap_Value;
	// --------------------------------------------------------------------------
       //   ENCODE
       // --------------------------------------------------------------------------
    // PID gain set if time is 2.



//    if (vicon_message_counter<=10){
// 	   pos_est.usec = 2;
// 	   pos_est.x=500;
// 	   pos_est.y=600;
// 	   pos_est.z=0;
//
// 	   pos_est.roll=600;
// 	   pos_est.pitch=500;
// 	   pos_est.yaw=0;
//    }
//
//    if (vicon_message_counter>=11 && vicon_message_counter<=20){
// 	   pos_est.usec = 3;
// 	   pos_est.x=592;
// 	   pos_est.y=616;
// 	   pos_est.z=642;
//
// 	   pos_est.roll=0;
// 	   pos_est.pitch=0;
// 	   pos_est.yaw=0;
//    }
//           printf("---%ld \n\n",pos_est.usec);
//       printf("%f \t  %f \t  %f \t %f \t  %f \t  %f \n\n",pos_est.x,pos_est.y,pos_est.z,pos_est.pitch,pos_est.roll, pos_est.yaw);

       mavlink_message_t message;

       mavlink_msg_vicon_position_estimate_encode(system_id, companion_id, &message, &pos_est);
//       mavlink_msg_sim_state_encode(system_id, companion_id, &message, &allposes)
       vicon_message_counter++;





       // --------------------------------------------------------------------------
       //   WRITE
       // --------------------------------------------------------------------------

       // do the write
       int len = 0;
       len = write_message(message);

       // check the write
       if ( len <= 0 )
               fprintf(stderr,"WARNING: could not send MOCAP value \n");
       //      else
       //              printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_ta

       return;
}



// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = udp_client->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK TCP PORT
	// --------------------------------------------------------------------------

//	if ( client->status != 1 ) // PORT_OPEN
//	{
//		fprintf(stderr,"ERROR: client port not open\n");
//		throw 1;
//	}
	if ( udp_client->status != 1 ) // PORT_OPEN
		{
			fprintf(stderr,"ERROR: client port not open\n");
			throw 1;
		}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

//	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------
//
//	printf("CHECK FOR MESSAGES\n");
//
//	while ( not current_messages.sysid )
//	{
//		if ( time_to_exit )
//			return;
//		usleep(500000); // check at 2Hz
//	}
//
//	printf("Found\n");
//
//	// now we know autopilot is sending messages
//	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
//	while ( not ( current_messages.time_stamps.local_position_ned &&
//				  current_messages.time_stamps.attitude            )  )
//	{
//		if ( time_to_exit )
//			return;
//		usleep(500000);
//	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
//		usleep(20000); // 50Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the client_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		// Print local position NED
		printf("NED_x: %f, NED_y: %f, NED_z: %f",current_messages.local_position_ned.x,current_messages.local_position_ned.y,current_messages.local_position_ned.z);
		usleep(100); // Read batches at 50Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;

	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	timeLastWrittenMsg = get_time_usec();
	while ( !time_to_exit )
	{
		usleep(2e4);   // Stream at 50HZ
//		write_gps_raw_int();

		write_mocap_floats();
		timeCurrentWrittenMsg = get_time_usec();
//		printf("\n\n\n Time took from last message: %f \n",(timeCurrentWrittenMsg-timeLastWrittenMsg)/1e6);
		timeLastWrittenMsg = get_time_usec();
	}
	printf("exit\n");
	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



