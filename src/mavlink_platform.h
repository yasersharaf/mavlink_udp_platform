/*
 * mavlink_platform.h
 *
 *  Created on: Aug 12, 2017
 *      Author: yaser
 */

#ifndef MAVLINK_PLATFORM_H_
#define MAVLINK_PLATFORM_H_

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "libraries/mavlink/v1.0/common/mavlink.h"
#include "libraries/autopilot_interface.h"
#include "libraries/client.h"

#include "libraries/gnuplot-iostream.h"

#include "libraries/NatNetLinux/NatNet.h"
#include "libraries/NatNetLinux/CommandListener.h"
#include "libraries/NatNetLinux/FrameListener.h"


using std::string;
using namespace std;



// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);
void quat2Euler(float qx,float qy,float qz,float qw,  float& roll, float& pitch, float& yaw);

void commands(Autopilot_Interface &autopilot_interface);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
void readOpts(int argc, char* argv[]);

// quit handler
std::vector <Autopilot_Interface> *autopilot_interface_quit;
std::vector <UDP_Client> *udp_client_quit;
//Client *client_quit;
void quit_handler( int sig );




#endif /* MAVLINK_PLATFORM_H_ */
