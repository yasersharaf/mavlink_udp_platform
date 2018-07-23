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
uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return (uint64_t)_time_stamp.tv_sec*1000000L + (uint64_t)_time_stamp.tv_usec;

}

//
//uint64_t get_time_since_epoch()
//{
//	return   get_time_usec()-platform_epoch;
//}
//
uint64_t get_time_since_epoch_embed_yaw(float mocap_yaw_,uint64_t platform_epoch_64_, float quad_roll_offset_, float quad_pitch_offset_)
{
//	quad_roll_offset_ = 0.054321;
//	quad_pitch_offset_ = 0.045678;
	cout<< "\n packing: "<<mocap_yaw_<<"\t"<<quad_roll_offset_<<"\t"<<quad_pitch_offset_<<"\n";
//	printf("mocapYaw: %llu\n",((uint64_t((mocap_yaw_+2*M_PI)*1e4)))*10);
//	return   ((uint64_t((mocap_yaw_+2*M_PI)*1e4)))*10;
	return   ((uint64_t((mocap_yaw_+2*M_PI)*1e4)))*10+ uint64_t(floor((1-quad_roll_offset_)*10000))*1e6+uint64_t(floor((1-quad_pitch_offset_)*10000))*1e11;
//	return   get_time_usec();
	//delay test
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

Autopilot_Interface::Autopilot_Interface(Client* client_,uint64_t platform_epoch_64_){
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
	platform_epoch_64 =  platform_epoch_64_;
}

Autopilot_Interface::Autopilot_Interface(UDP_Client& udp_client_,uint64_t platform_epoch_64_){
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
	std::cout<<udp_client_.port<<std::endl;

	udp_client = &udp_client_; // client port management object
	client = 0;
	std::cout<<udp_client->port<<std::endl;
	platform_epoch_64 =  platform_epoch_64_;


	target_mocap.x=0;
	target_mocap.y=0;
	target_mocap.z=0;
	target_mocap.roll=0;
	target_mocap.pitch=0;
	target_mocap.yaw=0;

	filtVx = 0.0;
	filtVy = 0.0;
	filtVz = 0.0;

	current_mocap_value.x=0;
	current_mocap_value.y=0;
	current_mocap_value.z=0;
	current_mocap_value.roll=0;
	current_mocap_value.pitch=0;
	current_mocap_value.yaw=0;

	previous_mocap_value = current_mocap_value;

	currentMocapRead = get_time_usec();


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

void
Autopilot_Interface::set_position_vicon_message(float x, float y, float z)
{
	target_mocap.usec = 3;
	target_mocap.x   = x;
	target_mocap.y   = y;
	target_mocap.z   = z;
	target_mocap.roll   = 1.0/20;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", target_mocap.x, target_mocap.x, target_mocap.x);

}

void
Autopilot_Interface::change_mode_then_arm_disarm(bool arm_disarm,bool is_mc)
{
	__mavlink_vicon_position_estimate_t mode_arm_disarm_packet;
	mode_arm_disarm_packet.usec =4;
	mode_arm_disarm_packet.x = arm_disarm==true ? 1.0:0.0;
	mode_arm_disarm_packet.y = is_mc==true ? 1.0:0.0;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", target_mocap.x, target_mocap.x, target_mocap.x);
    mavlink_message_t message;
    mavlink_msg_vicon_position_estimate_encode(system_id, companion_id, &message, &mode_arm_disarm_packet);
    int len = 0;
    for (int i = 0; i<3;i++){
        len = write_message(message);
        usleep(6e4);
    }
    printf("High priority set with result: %d\nHigh priority set with result: %d\nHigh priority set with result: %d\n",len,len,len);

}

void
Autopilot_Interface::land_command()
{
	target_mocap.x =  current_mocap_value.x;
	target_mocap.y =  current_mocap_value.y;
	target_mocap.z =  0;

	__mavlink_vicon_position_estimate_t pos_est = target_mocap;

	pos_est = target_mocap;
	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", target_mocap.x, target_mocap.y, target_mocap.z);
    mavlink_message_t message;
    mavlink_msg_vicon_position_estimate_encode(system_id, companion_id, &message, &pos_est);
    int len = 0;
    for (int i = 0; i<3;i++){
        len = write_message(message);
        usleep(6e4);
    }
    printf("Abort mission; target set with result: %d\nAbort mission; target set with result: %d\nAbort mission; target set with result: %d\n",len,len,len);

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
		if(message.msgid==MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE)
		{
			mavlink_msg_vicon_position_estimate_decode(&message, &received_mocap_value);
		}
		if(message.msgid==MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW)
		{
			mavlink_msg_hil_rc_inputs_raw_decode(&message, &received_raw_mocap_long_value);
			process_raw_long_mocap(&received_raw_mocap_long_value,&received_mocap_long_value);
		}

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
	current_mocap_long_value.usec = get_time_usec();
//	current_mocap_value.usec = get_time_since_epoch_embed_yaw(mocap_yaw, platform_epoch_64, quad_roll_offset, quad_pitch_offset);
//	uint64_t pp = floor(current_mocap_value.usec/100000000000);
//	uint64_t rr = floor((current_mocap_value.usec-pp*100000000000)/1000000);
//	uint64_t yy = current_mocap_value.usec-pp*100000000000- rr*1000000;
//	cout<< "coded val:  "<< current_mocap_value.usec<<"\t"<<1-pp/1e4<<"\t"<<1-rr/1e4<<"\t"<<yy/1e5-2*M_PI<<"\n";
	rev_process_float_long_mocap(&current_raw_mocap_long_value, &current_mocap_long_value);
	__mavlink_hil_rc_inputs_raw_t pos_est = current_raw_mocap_long_value;
	// --------------------------------------------------------------------------
       //   ENCODE
       // --------------------------------------------------------------------------
    // PID gain set if time is 2.



    if (vicon_message_counter<=10){
 	   pos_est.time_usec = 2;
 	   pos_est.chan1_raw = 1800;
 	   pos_est.chan2_raw = 700;
 	   pos_est.chan3_raw = pixhawkVersion;

 	   pos_est.chan4_raw=230;
 	   pos_est.chan5_raw=230;
 	   pos_est.chan6_raw=0;
    }

    if (vicon_message_counter>=11 && vicon_message_counter<=20){
    	pos_est.time_usec = target_mocap.usec;
    	pos_est.chan1_raw = target_mocap.x*1000;
    	pos_est.chan2_raw = target_mocap.y*1000;
    	pos_est.chan3_raw = target_mocap.z*1000;
    }
//    if (vicon_message_counter>=21 && vicon_message_counter<=30){
//    	attitude_gain.usec = 5;
//    	attitude_gain.x = 0.02;
//    	attitude_gain.y = 0.0;
//    	attitude_gain.z = 0.1;
//    	attitude_gain.roll = 0.02;
//    	attitude_gain.pitch = flipInitialAccel;
//    	attitude_gain.yaw = flipAngVel;
//    	pos_est = attitude_gain;
//    }

//       printf("---%ld \n\n",pos_est.usec);
//       printf("%f \t  %f \t  %f \t %f \t  %f \t  %f \n\n",pos_est.x,pos_est.y,pos_est.z,pos_est.pitch,pos_est.roll, pos_est.yaw);

       mavlink_message_t message;

       mavlink_msg_hil_rc_inputs_raw_encode(system_id, companion_id, &message, &pos_est);
       vicon_message_counter++;
//       printf("vicon message Counter: %f",vicon_message_counter);





       // --------------------------------------------------------------------------
       //   WRITE
       // --------------------------------------------------------------------------

       // do the write
       int len = 0;
       len = write_message(message);
       std::cout<<len<<std::endl;
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
	printf("Autopilot_Interface starting ...");
	int result;
	udp_client->connect();

	// --------------------------------------------------------------------------
	//   CHECK TCP/UDP PORT
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
		change_mode_then_arm_disarm(false,false);
		land_command();
		usleep(3.5e6);
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
		usleep(200); // Read batches at 50Hz
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
//		1/write_frequncy
		usleep(2.5e4);   // Stream at 20HZ

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

// ------------------------------------------------------------------------------
//   Process Long Message divide by 1000, cast as float
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
process_raw_long_mocap(mavlink_hil_rc_inputs_raw_t* raw_mocap_long_value,__mavlink_optitrack_position_estimate_t* mocap_long_value)
{

	mocap_long_value->usec = raw_mocap_long_value->time_usec;
	mocap_long_value->x = ((float)raw_mocap_long_value->chan1_raw)/1000;
	mocap_long_value->y = ((float)raw_mocap_long_value->chan2_raw)/1000;
	mocap_long_value->z = ((float)raw_mocap_long_value->chan3_raw)/1000; ///< RC channel 3 value, in microseconds
	mocap_long_value->Vx = ((float)raw_mocap_long_value->chan4_raw)/1000; ///< RC channel 4 value, in microseconds
	mocap_long_value->Vy = ((float)raw_mocap_long_value->chan5_raw)/1000; ///< RC channel 5 value, in microseconds
	mocap_long_value->Vz = ((float)raw_mocap_long_value->chan6_raw)/1000; ///< RC channel 6 value, in microseconds
	mocap_long_value->roll_rel = ((float)raw_mocap_long_value->chan7_raw)/1000; ///< RC channel 7 value, in microseconds
	mocap_long_value->pitch_rel = ((float)raw_mocap_long_value->chan8_raw)/1000; ///< RC channel 8 value, in microseconds
	mocap_long_value->yaw_abs = ((float)raw_mocap_long_value->chan9_raw)/1000; ///< RC channel 9 value, in microseconds
	mocap_long_value->target_x = ((float)raw_mocap_long_value->chan10_raw)/1000; ///< RC channel 10 value, in microseconds
	mocap_long_value->target_y = ((float)raw_mocap_long_value->chan11_raw)/1000; ///< RC channel 11 value, in microseconds
	mocap_long_value->target_z = ((float)raw_mocap_long_value->chan12_raw)/1000; ///< RC channel 12 value, in microseconds
	mocap_long_value->target_yaw_rel = (3.0/2000)*(float)raw_mocap_long_value->rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%
	printf("yaw:    %f \n",mocap_long_value->x);
}


// ------------------------------------------------------------------------------
//   Process Long Message divide by 12000, cast as float
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
rev_process_float_long_mocap(mavlink_hil_rc_inputs_raw_t* raw_mocap_long_value,__mavlink_optitrack_position_estimate_t* mocap_long_value)
{

	raw_mocap_long_value->time_usec = mocap_long_value->usec;
	raw_mocap_long_value->chan1_raw = mocap_long_value->x*1000;
	raw_mocap_long_value->chan2_raw = mocap_long_value->y*1000;
	raw_mocap_long_value->chan3_raw = mocap_long_value->z*1000; ///< RC channel 3 value, in microseconds
	raw_mocap_long_value->chan4_raw = mocap_long_value->Vx*1000; ///< RC channel 4 value, in microseconds
	raw_mocap_long_value->chan5_raw = mocap_long_value->Vy*1000; ///< RC channel 5 value, in microseconds
	raw_mocap_long_value->chan6_raw = mocap_long_value->Vz*1000; ///< RC channel 6 value, in microseconds
	raw_mocap_long_value->chan7_raw = mocap_long_value->roll_rel*1000; ///< RC channel 7 value, in microseconds
	raw_mocap_long_value->chan8_raw = mocap_long_value->pitch_rel*1000; ///< RC channel 8 value, in microseconds
	raw_mocap_long_value->chan9_raw = mocap_long_value->yaw_abs*1000; ///< RC channel 9 value, in microseconds
	raw_mocap_long_value->chan10_raw = mocap_long_value->target_x*1000; ///< RC channel 10 value, in microseconds
	raw_mocap_long_value->chan11_raw = mocap_long_value->target_y*1000; ///< RC channel 11 value, in microseconds
//	mocap_long_value->target_z = 1.7;
	raw_mocap_long_value->chan12_raw = mocap_long_value->target_z*1000; ///< RC channel 12 value, in microseconds
	raw_mocap_long_value->rssi = mocap_long_value->target_yaw_rel*2000/3; ///< Receive signal strength indicator, 0: 0%, 255: 100%
	printf("sent rellllllllllllllllllllllllllllllllll yyyyyyyyyyyaw:    %f \n",180/M_PI*(3.0/2000)*(float)raw_mocap_long_value->rssi);
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



