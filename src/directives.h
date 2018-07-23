/*
 * directives.h
 *
 *  Created on: Aug 15, 2017
 *      Author: yaser
 */

#ifndef DIRECTIVES_H_
#define DIRECTIVES_H_

# define M_PI  3.14159265358979323846


# define APM_IP1 "192.168.0.202"
# define APM_PORT1 5678

# define APM_IP2 "192.168.0.150"
# define APM_PORT2 3456


# define APM_IP3 "192.168.0.203"
# define APM_PORT3 6789

# define APM_IP4 "127.0.0.1"
# define APM_PORT4 4567


//# define APM_IP1 "192.168.0.102"
//# define APM_PORT1 6789
//
//# define APM_IP2 "192.168.0.100"
//# define APM_PORT2 7890
//
//# define APM_IP3 "192.168.0.103"
//# define APM_PORT3 5678



#define usingUDP //If using UDP (toward multidrone)
//#define usingTCP //If using TCP
//#define usingSerial //If using Serial



#define PORT_OPEN   1;
#define PORT_CLOSED 0;
#define PORT_ERROR -1;

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"


typedef struct __mavlink_optitrack_position_estimate_t
{
 uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 float x; ///< RC channel 1 value, in microseconds
 float y; ///< RC channel 2 value, in microseconds
 float z; ///< RC channel 3 value, in microseconds
 float Vx; ///< RC channel 4 value, in microseconds
 float Vy; ///< RC channel 5 value, in microseconds
 float Vz; ///< RC channel 6 value, in microseconds
 float roll_rel; ///< RC channel 7 value, in microseconds
 float pitch_rel; ///< RC channel 8 value, in microseconds
 float yaw_abs; ///< RC channel 9 value, in microseconds
 float target_x; ///< RC channel 10 value, in microseconds
 float target_y; ///< RC channel 11 value, in microseconds
 float target_z; ///< RC channel 12 value, in microseconds
 float target_yaw_rel; ///< Receive signal strength indicator, 0: 0%, 255: 100%
} mavlink_optitrack_position_estimate_t;


#endif /* DIRECTIVES_H_ */
