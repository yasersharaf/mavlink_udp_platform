//============================================================================
// Name        : mavlink_platform.cpp
// Author      : Yas
// Version     :
// Copyright   : 
// Description : A platform for multi-unmanned vehicles in C++
//============================================================================


#include "mavlink_platform.h"


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <boost/program_options.hpp>
#include <time.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <utility>
#include "directives.h"



timeval current_time;
float elapsed_time = 0;

unsigned int historyLength = 1;
std::vector<float> x_hist;
std::vector<float> y_hist;
std::vector<float> z_hist;
std::vector<std::pair<double,double>> data;
std::vector<std::pair<double,double>> data2;


class Globals {
public:

	// Parameters read from the command line
	static uint32_t localAddress;
	static uint32_t serverAddress;


	// State of the main() thread.
	static bool run;
};
uint32_t Globals::localAddress = 0;
uint32_t Globals::serverAddress = 0;
bool Globals::run = false;

// End the program gracefully.
void terminate(int) {
	// Tell the main() thread to close.
	Globals::run = false;
}



// This thread loop just prints frames as they arrive.
void printFrames(Autopilot_Interface& api1,Autopilot_Interface& api2 ) {
	bool valid;
	MocapFrame frame;
	Globals::run = true;
	int sendCounter = 0;
//	  Gnuplot gp;
//	  gp.close();
	while (Globals::run) {
		while (true) {
			sendCounter++;
//			printf("__________________thread running________________________\n");


			api1.current_mocap_value.x = 1;
			api1.current_mocap_value.y = 2;
			api1.current_mocap_value.z = 3;
			api1.current_mocap_value.pitch = 	4;
			api1.current_mocap_value.roll  = 	5;
			api1.current_mocap_value.yaw 	 =  6;

			api2.current_mocap_value.x = 10;
			api2.current_mocap_value.y = 20;
			api2.current_mocap_value.z = 30;
			api2.current_mocap_value.pitch = 	40;
			api2.current_mocap_value.roll  = 	50;
			api2.current_mocap_value.yaw 	 =  60;


			usleep(400000);
		}

		// Sleep for a little while to simulate work :)

	}
}

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top(int argc, char **argv) {


	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------
#ifdef usingS
		// Default input arguments
	#ifdef __APPLE__
		char *uart_name = (char*)"/dev/tty.usbmodem1";
	#else
		char *uart_name = (char*) "/dev/ttyUSB0";
	//	char *uart_name = (char*) "/dev/ttyUSB1";
	//	char *uart_name = (char*) "/dev/ttyACM0";
	#endif
		int baudrate = 57600;
	//	int baudrate = 115200;
#endif

	// --------------------------------------------------------------------------
	//   PORT and THREAD for READ AND WRITE STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
#ifdef usingSerial
	Serial_Port serial_port(uart_name, baudrate);
	Autopilot_Interface autopilot_interface(&serial_port);
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT, quit_handler);
#elif defined(usingTCP)
	Client client("128.180.103.41", APM_PORT);
	Autopilot_Interface autopilot_interface(&client);
	client_quit = &client;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT, quit_handler);
#else
//	std::vector<UDP_Client> udp_clients;
//	udp_clients.push_back(UDP_Client udpclient("192.168.0.101", APM_PORT));
//	udp_clients.push_back(UDP_Client("192.168.0.103", APM_PORT));
//	udp_clients.push_back(UDP_Client("192.168.0.104", APM_PORT));
	UDP_Client udp_client1((std::string)APM_IP1,APM_PORT1,1);
	Autopilot_Interface autopilot_interface1(&udp_client1);
	autopilot_interface_quit = &autopilot_interface1;

	UDP_Client udp_client2((std::string)APM_IP2,APM_PORT2,2);
	Autopilot_Interface autopilot_interface2(&udp_client2);
//	autopilot_interface_quit = &autopilot_interface;


	signal(SIGINT, quit_handler);
#endif

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
//	client.open_connection();
	autopilot_interface1.start();
	autopilot_interface2.start();

	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------
	printf("-----------------starting5-----------------\n");
	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
//	commands(autopilot_interface);

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */

	// This infinite loop simulates a "worker" thread that reads the frame
	// buffer each time through, and exits when ctrl-c is pressed.
	printf("-----------------preprint-----------------\n");
	printFrames(autopilot_interface1,autopilot_interface2);
	//timeStats(frameListener);


	autopilot_interface1.stop();
	autopilot_interface2.stop();

//	client.close_connection();

	return 0;

}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void commands(Autopilot_Interface &api) {

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command

	// Example 1 - Set Velocity
//	set_velocity( -1.0       , // [m/s]
//				  -1.0       , // [m/s]
//				   0.0       , // [m/s]
//				   sp        );

	// Example 2 - Set Position
	set_position(ip.x - 5.0, // [m]
	ip.y - 5.0, // [m]
	ip.z, // [m]
			sp);

	// Example 1.2 - Append Yaw Command
	set_yaw(ip.yaw, // [rad]
			sp);

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i = 0; i < 8; i++) {
		mavlink_local_position_ned_t pos =
				api.current_messages.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i,
				pos.x, pos.y, pos.z);
//		sleep(0.2);
	}

	printf("\n");

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands

	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf(
			"Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z);

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf(
			"Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
	printf("    ap time:     %llu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc,
			imu.zacc);
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro,
			imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
	printf("    baro:        %f (mBar) \n", imu.abs_pressure);
	printf("    altitude:    %f (m) \n", imu.pressure_alt);
	printf("    temperature: %f C \n", imu.temperature);

	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate) {

	// string for command line usage
	const char *commandline_usage =
			"usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n", commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n", commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n", commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}

// Set the global addresses from the command line.
void readOpts(int argc, char* argv[]) {
	namespace po = boost::program_options;

	po::options_description desc(
			"simple-example: demonstrates using NatNetLinux\nOptions");
	desc.add_options()("help", "Display help message")("local-addr,l",
			po::value<std::string>(), "Local IPv4 address")("server-addr,s",
			po::value<std::string>(), "Server IPv4 address");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (argc < 5 || vm.count("help") || !vm.count("local-addr")
			|| !vm.count("server-addr")) {
		std::cout << desc << std::endl;
		exit(1);
	}

	Globals::localAddress = inet_addr(
			vm["local-addr"].as<std::string>().c_str());
	Globals::serverAddress = inet_addr(
			vm["server-addr"].as<std::string>().c_str());
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler(int sig) {
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	} catch (int error) {
	}

	// serial port
	try {
		client_quit->handle_quit(sig);
	} catch (int error) {
	}

	// end program here
	exit(0);

}
void quat2Euler(float qx,float qy,float qz,float qw,  float& roll, float& pitch, float& yaw){

	double r11 = 2*(qy*qw + qx*qz);
	double r12 = qx*qx - qy*qy - qz*qz + qw*qw;
	double r21 =-2*(qz*qw - qx*qy);
	double r31 = 2*(qy*qz + qx*qw);
	double r32 = qx*qx - qy*qy + qz*qz - qw*qw;
	//-180 to 180 yaw
	yaw   = (float)180/M_PI*std::atan2( r11, r12 );
	//-90 to 90 pitch
	pitch = (float)-180/M_PI*std::asin( r21 );
	//-180 to 180 roll
	roll   = (float)180 - 180/M_PI*std::atan2( r31, r32 );



}
// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv) {
	// This program uses throw, wrap one big try/catch here
	try {
		int result = top(argc, argv);
		return result;
	}

	catch (int error) {
		fprintf(stderr, "mavlink_control threw exception %i \n", error);
		return error;
	}

}

