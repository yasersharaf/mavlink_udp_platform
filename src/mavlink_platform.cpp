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
#include "libraries/gnuplot-iostream.h"
#include <fstream>

//for autIP
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/if_link.h>

//threads for printFrames:
#include <thread>         // std::thread

timeval current_time;
float elapsed_time = 0;

unsigned int historyLength = 1;
std::vector<float> x_hist;
std::vector<float> y_hist;
std::vector<float> z_hist;
std::vector<std::pair<double,double>> data;
std::vector<std::pair<double,double>> data2;
std::vector<std::pair<double,double>> dataxy;

std::ofstream logFile;
uint64_t platform_epoch = 0;



class Globals {
public:

	// Parameters read from the command line
	static uint32_t localAddress;
	static uint32_t serverAddress;


	// State of the main() thread.
	static bool run;
	static bool haveServerIP;
	static bool haveLocalIP;
	static bool obtainedIP;
};
uint32_t Globals::localAddress = 0;
uint32_t Globals::serverAddress = 0;
bool Globals::run = false;
bool Globals::haveServerIP = false;
bool Globals::haveLocalIP = false;
bool Globals::obtainedIP = true;

// End the program gracefully.
void terminate(int) {
	// Tell the main() thread to close.
	Globals::run = false;
}

std::string getIP(char* netType)
{
	struct ifaddrs *ifaddr, *ifa;
	int family, s, n;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1) {
		perror("getifaddrs");
		printf("Failed to obtain ip!\n");
	}

	/* Walk through linked list, maintaining head pointer so we
              can free list later */

	for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
		if (ifa->ifa_addr == NULL)
			continue;

		family = ifa->ifa_addr->sa_family;


		/* For an AF_INET* interface address, display the address */

		if (family == AF_INET ) {

			/* Display interface name and family (including symbolic
	                  form of the latter for the common families) */
			if (strncmp(netType,ifa->ifa_name,1)==0){
//				printf("this is the case:");
				printf("%-8s \n", ifa->ifa_name);
				s = getnameinfo(ifa->ifa_addr,
						(family == AF_INET) ? sizeof(struct sockaddr_in) :
								sizeof(struct sockaddr_in6),
								host, NI_MAXHOST,
								NULL, 0, NI_NUMERICHOST);
				freeifaddrs(ifaddr);
				return std::string(host);

				if (s != 0) {
					printf("Failed to obtain ip Automatically!\n");
				}
//				printf("\t\taddress: <%s>\n", host);
			}
		}
	}
	return "127.0.0.1";
}

// This thread loop just prints frames as they arrive.
void printFrames(FrameListener& frameListener,std::vector<Autopilot_Interface>& api ) {

	bool valid;
	MocapFrame frame;
	std::pair<MocapFrame, struct timespec> frameAndTime;
	struct timespec mocap_current_ts;
	struct timespec mocap_previous_ts;
	Globals::run = true;
    Gnuplot gp;
    Gnuplot gpxy;
	gpxy << "set xrange [-4:4]\nset yrange [-5:5]\n";
	gpxy << "set size ratio -1\n";


    int readFromMocapCounter = 0; //for gnu
    int rollMemCounter = 0;
    int pitchMemCounter = 0;

//	  gp.close();
    std::string fileName  = "log"+std::to_string(get_time_usec())+".txt";
	logFile.open(fileName);

//	printf("=================================================== \t Log file opened!===================================================\n=========================================================================================================================================================\n=========================================================================================================================================================\n");




	while (Globals::run) {
		//TODO:::
//		printf("=================================================== \t Running!===================================================\n");

		frameAndTime = frameListener.pop(&valid);

//		MocapFrame frame(frameListener.pop(&valid).first);
		// Quit if the listener has no more frames.
		if (!valid){
//			printf("No frame yet.\n");
			usleep(1e3);
//			printf("=================================================== \t Retrying!===================================================\n");

			continue;
		}
		else{
//			printf("=================================================== \t Got it!===================================================\n");
		}
		frame = frameAndTime.first;
		mocap_current_ts = frameAndTime.second;


		std::vector<RigidBody> const& rBodies = frame.rigidBodies();
//		rBodies.
//		uint16_t numerOfBodies = rBodies.size();
//TODO:::
		if (rBodies.size()<api.size()){
			fprintf (stderr, "\n\n Rigid bodies less than ap interfaces!\n\n\n");
			std::cout<<"num ap inter: "<<api.size()<<"num rigid bodies: "<<rBodies.size()<<std::endl;
			break;
		}
		else if (rBodies.size()<api.size()){
			fprintf (stderr, "\n\n Rigid bodies more than ap interfaces!\n\n\n");
			break;
		}

		float roll= 0;
		float pitch= 0;
		float yaw = 0;

		float ax_body_hover = 0;
		float ay_body_hover = 0;
		float az_body_hover = 0;

		float ax_hover = 0;
		float ay_hover = 0;
		float az_hover = 0;

		float dt =0;
		float dt2 =0;
		uint64_t u_dt = 0;
		dt2 =  (std::abs(static_cast<float>(mocap_current_ts.tv_sec-mocap_previous_ts.tv_sec)*1000.f
	                + (static_cast<float>(mocap_current_ts.tv_nsec)-static_cast<float>(mocap_previous_ts.tv_nsec))/1000000))/1000;
		if (dt2>0.9){
			fprintf (stderr, "Time wrapped!\n");
			printf("Time wrapped\n");
			std::cout<<"actual:"<<static_cast<float>(mocap_current_ts.tv_sec)<<"\t"<<static_cast<float>(mocap_previous_ts.tv_sec)<<"\n";
			std::cout<<"diff"<<static_cast<float>(mocap_current_ts.tv_sec-mocap_previous_ts.tv_sec)*1.f<<"\t"<<(static_cast<float>(mocap_current_ts.tv_nsec)-static_cast<float>(mocap_previous_ts.tv_nsec))/1e9<<"\n";
			fprintf (stderr, "Time wrapped!\n");
		}
		if (readFromMocapCounter == 0){
			mocap_previous_ts = mocap_current_ts;
			readFromMocapCounter++;
			continue;

		}
		if (dt2>3e-2){
//			mocap_previous_ts = mocap_current_ts;
			cout<<"\t\tdt2 = "<<dt2 <<"old data; getting more data at:"<<get_time_usec() <<"\n";
			cout<<"\t\tdt2 = "<<dt2 <<"old data; getting more data \n";
			cout<<"\t\tdt2 = "<<dt2 <<"old data; getting more data \n";
			cout<<"\t\tdt2 = "<<dt2 <<"old data; getting more data \n";
			cout<<"\t\tdt2 = "<<dt2 <<"old data; getting more data \n";
//			continue;
		}
		if (dt2<6e-3){
			dt2 = 8.3e-3;
		}
		else if(dt>2e-2){
			dt = 2e-2;
		}
		mocap_previous_ts = mocap_current_ts;
		for(uint16_t api_ctr =0; api_ctr<api.size();api_ctr++){
//			printf("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t %f\n",api[api_ctr].target_mocap.z);
//TODO:::
			quat2Euler(rBodies[api_ctr].orientation().qx,rBodies[api_ctr].orientation().qy,rBodies[api_ctr].orientation().qz,rBodies[api_ctr].orientation().qw, roll, pitch, yaw);
			api[api_ctr].mocap_yaw = yaw;

//			std::cout<<"yaw: "<<api[api_ctr].current_mocap_value.usec<<" yaw: "<<yaw<<std::endl;
//			api[api_ctr].current_mocap_value.x = cos(M_PI/180*yaw)*rBodies[api_ctr].location().z +  sin(M_PI/180*yaw)*rBodies[api_ctr].location().x;
//			api[api_ctr].current_mocap_value.y = -sin(M_PI/180*yaw)*rBodies[api_ctr].location().z +  cos(M_PI/180*yaw)*rBodies[api_ctr].location().x;
//			api[api_ctr].current_mocap_value.z = rBodies[api_ctr].location().y;
			api[api_ctr].current_mocap_value.x = rBodies[api_ctr].location().z;
			api[api_ctr].current_mocap_value.y = rBodies[api_ctr].location().x;
			api[api_ctr].current_mocap_value.z = rBodies[api_ctr].location().y;

			if (api[api_ctr].previous_mocap_value.x == 0){
				printf("resettig prev mocap val\n");
				printf("resettig prev mocap val\n");
				api[api_ctr].previous_mocap_value = api[api_ctr].current_mocap_value;
			}

//			std::cout<<"mocapval: "<<"\t"<<api[api_ctr].previous_mocap_value.x<<"\t"<<api[api_ctr].current_mocap_value.x<<std::endl;

//						api[api_ctr].current_mocap_value.usec = get_time_usec();
//						api[api_ctr].current_mocap_value.x = rand();
//						api[api_ctr].current_mocap_value.y = rand();
//						api[api_ctr].current_mocap_value.z = rand();
			u_dt = get_time_usec() - api[api_ctr].previous_mocap_value.usec;
			dt = (float)(u_dt/1e6);

//			printf("dt = %f \t dt2 = %f \n",dt,dt2);
			// filter

			ax_body_hover = tan(-(roll -api[api_ctr].mocap_roll_offset))*9.81;
			ay_body_hover = tan(pitch - api[api_ctr].mocap_pitch_offset)*9.81;

			ax_hover = cosf(yaw)*ax_body_hover-sinf(yaw)*ay_body_hover;
			ay_hover = sinf(yaw)*ax_body_hover+cosf(yaw)*ay_body_hover;
//			std::cout<<"accel:\t"<<ax_body_hover<<"\t"<<ay_body_hover<<"\t"<<ax_hover<<"\t"<<ay_hover<<"\t"<<api[api_ctr].filtVx<<"\t"<<api[api_ctr].filtVy<<"\n";


			api[api_ctr].noisyVx = (api[api_ctr].current_mocap_value.x-api[api_ctr].previous_mocap_value.x)/dt2;
			api[api_ctr].noisyVy = (api[api_ctr].current_mocap_value.y-api[api_ctr].previous_mocap_value.y)/dt2;
			api[api_ctr].noisyVz = (api[api_ctr].current_mocap_value.z-api[api_ctr].previous_mocap_value.z)/dt2;


			api[api_ctr].filtVx = api[api_ctr].noisyVx*0.3+0.7*(api[api_ctr].filtVx+(1/120)*constrain_float((api[api_ctr].noisyVx-api[api_ctr].filtVx)*120,
					tan((roll)-0.02)*9.81,tan((roll)+0.02)*9.81));
			api[api_ctr].filtVy = api[api_ctr].noisyVy*0.3+0.7*(api[api_ctr].filtVy+(1/120)*constrain_float((api[api_ctr].noisyVy-api[api_ctr].filtVy)*120,
					tan(-pitch -0.02)*9.81,tan(-pitch+0.02)*9.81));

			api[api_ctr].current_mocap_value.pitch = api[api_ctr].filtVx;
			api[api_ctr].current_mocap_value.roll  = api[api_ctr].filtVy;
			api[api_ctr].current_mocap_value.yaw =  0.8*api[api_ctr].current_mocap_value.yaw   + 0.2*api[api_ctr].noisyVz;

			api[api_ctr].smoothVx = api[api_ctr].smoothVx*0.97+0.03*api[api_ctr].filtVx;
			api[api_ctr].smoothVy = api[api_ctr].smoothVy*0.97+0.03*api[api_ctr].filtVy;

			api[api_ctr].quadFiltRoll  = api[api_ctr].quadFiltRoll *0.97+ 0.03*api[api_ctr].received_mocap_value.x;
			api[api_ctr].quadFiltPitch = api[api_ctr].quadFiltPitch*0.97+ 0.03*api[api_ctr].received_mocap_value.roll;

			api[api_ctr].mocapAccelRealTimeX = std::atan((api[api_ctr].smoothVx_prev-api[api_ctr].smoothVx)*120/9.81);
			api[api_ctr].mocapAccelRealTimeY = std::atan((api[api_ctr].smoothVy-api[api_ctr].smoothVy_prev)*120/9.81);

			if (readFromMocapCounter>2500 && abs(api[api_ctr].received_mocap_value.yaw + 2)< 1e-5){
				api[api_ctr].quad_roll_offset  = constrain_float(-(-api[api_ctr].mocapAccelRealTimeX +  api[api_ctr].quadFiltRoll),-0.05,0.05);
				api[api_ctr].quad_pitch_offset = constrain_float(-(-api[api_ctr].mocapAccelRealTimeY + api[api_ctr].quadFiltPitch),-0.05,0.05);
//				api[api_ctr].mocap_roll_offset = 0.0;
//				api[api_ctr].mocap_pitch_offset = 0.0;
//
//				api[api_ctr].quad_roll_offset  = 0.0;
//				api[api_ctr].quad_pitch_offset = 0.0;
//
//
//				api[api_ctr].quad_roll_offset  = api[api_ctr].quad_roll_offset*0.5 + 0.1*(roll -api[api_ctr].mocap_roll_offset   +  api[api_ctr].received_mocap_value.x);
//				api[api_ctr].quad_pitch_offset = api[api_ctr].quad_pitch_offset*0.9+ 0.1*(pitch - api[api_ctr].mocap_pitch_offset + api[api_ctr].received_mocap_value.roll);
				printf("\n\t\t\t\t\t\t\t\t\t\t\tfilt: %d , %f , %f \n",readFromMocapCounter,
						abs(api[api_ctr].received_mocap_value.yaw + 2));

			}


			else{
				printf("\n\t\t\t\t\t\t\t\t\t\t\tno filt: %d , %f , %f \n",readFromMocapCounter,
						abs(api[api_ctr].received_mocap_value.yaw + 2));
			}
			cout<<"preprocess: "<<api[api_ctr].quad_roll_offset<<"\t"<<api[api_ctr].quad_pitch_offset<<"\n";


			cout<<"postprocess: "<<api[api_ctr].quad_roll_offset<<"\t"<<api[api_ctr].quad_pitch_offset<<"\n";

			logFile<<api_ctr<<"\t"<<api[api_ctr].received_mocap_value.usec<<"\t"<<api[api_ctr].received_mocap_value.x<<"\t"<<api[api_ctr].received_mocap_value.y<<"\t"<<api[api_ctr].received_mocap_value.z<<"\t"<<api[api_ctr].received_mocap_value.roll<<"\t"<<api[api_ctr].received_mocap_value.pitch<<"\t"<<api[api_ctr].received_mocap_value.yaw
					<<"\t"<<roll<<"\t"<<pitch<<"\t"<<api[api_ctr].current_mocap_value.pitch<<"\t"<<api[api_ctr].current_mocap_value.roll<<"\t"<<api[api_ctr].filtVx<<"\t"<<api[api_ctr].filtVy
					<<"\t"<<dt2<<"\t"<<api[api_ctr].current_mocap_value.x<<"\t"<<api[api_ctr].current_mocap_value.y<<"\t"<<api[api_ctr].current_mocap_value.z<<"\t"<<get_time_usec()<<"\t"<<yaw
					<<"\t"<<(api[api_ctr].current_mocap_value.x-api[api_ctr].previous_mocap_value.x)/dt2<<"\t"<<(api[api_ctr].current_mocap_value.y-api[api_ctr].previous_mocap_value.y)/dt2
					<<"\t"<< api[api_ctr].smoothVx<<"\t"<<api[api_ctr].smoothVy
					<<"\t"<<api[api_ctr].quadFiltRoll<<"\t"<<api[api_ctr].quadFiltPitch
					<<"\t"<<api[api_ctr].mocapAccelRealTimeX<<"\t"<<api[api_ctr].mocapAccelRealTimeY
					<<"\t"<<api[api_ctr].quad_roll_offset<<"\t"<<api[api_ctr].quad_pitch_offset
					<<"\t"<<api[api_ctr].mocap_roll_offset<<"\t"<<api[api_ctr].mocap_pitch_offset
					<<"\n";
//			std::cout<<"apival:"<<api_ctr<<"\t"<<api[api_ctr].received_mocap_value.usec<<"\t"<<api[api_ctr].received_mocap_value.x<<"\t"<<api[api_ctr].received_mocap_value.y<<"\t"<<api[api_ctr].received_mocap_value.z<<"\t"<<api[api_ctr].received_mocap_value.roll<<"\t"<<api[api_ctr].received_mocap_value.pitch<<"\t"<<api[api_ctr].received_mocap_value.yaw<<"\t"<<"\n";
			api[api_ctr].previous_mocap_value = api[api_ctr].current_mocap_value;
			api[api_ctr].smoothVx_prev = api[api_ctr].smoothVx;
			api[api_ctr].smoothVy_prev = api[api_ctr].smoothVy;

			api[api_ctr].previous_mocap_value.usec = get_time_usec();

			//plottingdata
			if (readFromMocapCounter%11==0){

//				data.emplace_back(readFromMocapCounter,api[api_ctr].current_mocap_value.x);
//				data2.emplace_back(readFromMocapCounter,api[api_ctr].current_mocap_value.y);
//				dataxy.emplace_back(rBodies[api_ctr].location().z,rBodies[api_ctr].location().x);
//
//
//				if(data.size()>500){
//				data.erase(data.begin());
//				data2.erase(data2.begin());
//				dataxy.erase(dataxy.begin());
//				}
//				gp << "set grid;set autoscale fix;plot'-' tit 'X' with line,'-' tit 'Y' with line\n";
//				gpxy << "set grid;set autoscale fix;plot'-' tit 'XY' \n";
//				gp.send1d(data);
//				gp.send1d(data2);
//				gpxy.send1d(dataxy);
			}
		}
		usleep(100);
		readFromMocapCounter++;

	}
	printf("\nEnded autopilot belief updater!\n");
}

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top(int argc, char **argv) {

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------
#ifdef usingSerial
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


		unsigned char natNetMajor  = 3;
		unsigned char natNetMinor = 0;

		// Sockets
		int sdCommand;
		int sdData;
		// Set addresses
		readOpts(argc, argv);
		// Use this socket address to send commands to the server.

		printf("trying\n");
		struct sockaddr_in serverCommands = NatNet::createAddress(
				Globals::serverAddress, NatNet::commandPort);

		if (Globals::haveServerIP == true){
			sdCommand = NatNet::createCommandSocket(Globals::localAddress);

			if (sdCommand != -1){

				// Start the CommandListener in a new thread.
				CommandListener commandListener(sdCommand);
				commandListener.start();

				// Send a ping packet to the server so that it sends us the NatNet version
				// in its response to commandListener.
				NatNetPacket ping = NatNetPacket::pingPacket();
				ping.send(sdCommand, serverCommands);

				// Wait here for ping response to give us the NatNet version.
				commandListener.getNatNetVersion(natNetMajor, natNetMinor);

				commandListener.stop();
				commandListener.join();
			}
			close(sdCommand);
		}
		//TODO:::
		// Create sockets
		sdData = NatNet::createDataSocket(Globals::localAddress);
		// Start up a FrameListener in a new thread.
		FrameListener frameListener(sdData, natNetMajor, natNetMinor);
		printf("start 1");
		frameListener.start();
		usleep(2e5);
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
	std::vector<UDP_Client> udp_clients;
	udp_clients.push_back(UDP_Client((std::string)APM_IP1,APM_PORT1,1,platform_epoch));
//	udp_clients.push_back(UDP_Client((std::string)APM_IP2,APM_PORT2,2,platform_epoch));
//	udp_clients.push_back(UDP_Client((std::string)APM_IP3,APM_PORT3,3,platform_epoch));
//	udp_clients.push_back(UDP_Client((std::string)APM_IP4,APM_PORT4,4,platform_epoch));

	std::vector<Autopilot_Interface> autopilot_interfaces;
	for(uint8_t udp_ctr =0; udp_ctr<udp_clients.size();udp_ctr++){
		autopilot_interfaces.push_back(Autopilot_Interface(udp_clients[udp_ctr],platform_epoch));

	}

	autopilot_interface_quit = &autopilot_interfaces;
	udp_client_quit = &udp_clients;

	signal(SIGINT, quit_handler);
#endif

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */

//	client.open_connection();
	autopilot_interfaces[0].set_position_vicon_message(1.0 , 0.0, 1.7);
//	autopilot_interfaces[0].set_position_vicon_message(2.0 , 0.0, 1.7);
	autopilot_interfaces[1].set_position_vicon_message(0.75,  0.0, 1.7);
	autopilot_interfaces[2].set_position_vicon_message(0.75 ,-2.5, 1.7);
	for(uint8_t udp_ctr =0; udp_ctr<udp_clients.size();udp_ctr++){
		autopilot_interfaces[udp_ctr].start();
	}

	//TODO:::

	std::thread thisthread (printFrames,std::ref(frameListener),std::ref(autopilot_interfaces));

	usleep(5e6);

	for(uint8_t udp_ctr =0; udp_ctr<udp_clients.size();udp_ctr++){
		autopilot_interfaces[udp_ctr].change_mode_then_arm_disarm(true,true);
	}

	while(1){
		usleep(1e6);
		printf("running");
	}
//	timeStats(frameListener);



	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------
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

	for(uint8_t udp_ctr =0; udp_ctr<udp_clients.size();udp_ctr++){
		autopilot_interfaces[udp_ctr].change_mode_then_arm_disarm(false,false);
		autopilot_interfaces[udp_ctr].stop();
	}
	thisthread.join();
	//TODO:::
	// Wait for threads to finish.
	frameListener.stop();
	frameListener.join();

	// close data and command sockets
	close(sdData);

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


	if (!vm.count("server-addr")){
		usleep (2e5);
		printf("Warning: No server address specified!\n");
		Globals::haveServerIP = false;
		usleep (2e5);
	}
	else{
		Globals::haveServerIP = true;
	}


	if (!vm.count("local-addr")){
		usleep (2e5);
		Globals::haveLocalIP = false;
		printf("Warning: No local address specified!\n");
		usleep (2e5);
		printf("Trying Ethernet ...\n");
		usleep (2e5);
		std::string  ethernetIP = getIP("e");
		if (strcmp(ethernetIP.c_str(),"127.0.0.1")==0){
			printf("Trying WIFI ...\n");
			std::string  wifiIP = getIP("w");
			if (strcmp(wifiIP.c_str(),"127.0.0.1")==0){
				Globals::obtainedIP =false;
			}
			Globals::localAddress = inet_addr(wifiIP.c_str());
			std::cout<<wifiIP<<std::endl;
		}
		else{
			std::cout<<ethernetIP<<std::endl;
			Globals::localAddress = inet_addr(ethernetIP.c_str());
		}
		usleep (2e5);

	}
	else{
		Globals::haveLocalIP = true;
	}


	if (argc < 5 || vm.count("help") || !vm.count("local-addr")
			|| !vm.count("server-addr")) {
		std::cout << desc << std::endl;
	}
	if (!Globals::obtainedIP){
		printf("exiting\n");
		exit(1);
	}
	if (Globals::haveServerIP){
		Globals::localAddress = inet_addr(
				vm["local-addr"].as<std::string>().c_str());
	}
	if (Globals::haveServerIP){
		Globals::serverAddress = inet_addr(
				vm["server-addr"].as<std::string>().c_str());
	}
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler(int sig) {
	printf("\n");
	printf("TERMINATING AT USER REQUEST...\n");
	printf("\n");

	printf("Closing File...\n");
	logFile.close();

	// autopilot interface
	try {
		for(uint16_t api_ctr = 0; api_ctr<autopilot_interface_quit->size();api_ctr++){
			(autopilot_interface_quit->at(api_ctr)).handle_quit(sig);
		}
	} catch (int error) {
	}

	// serial port
	try {
		for(uint16_t udp_ctr = 0; udp_ctr<autopilot_interface_quit->size();udp_ctr++){
			(udp_client_quit->at(0)).handle_quit(sig);
		}
	} catch (int error) {
	}

	// end program here
	exit(0);

}
void quat2Euler(float qx,float qy,float qz,float qw,  float& roll, float& pitch, float& yaw){

//	printf("q: %f\t%f\t%f\t%f\t",qx,qy,qz,qw);
	double r11 = 2*(qy*qw + qx*qz);
	double r12 = qx*qx - qy*qy - qz*qz + qw*qw;
	double r21 =-2*(qz*qw - qx*qy);
	double r31 = 2*(qy*qz + qx*qw);
	double r32 = qx*qx - qy*qy + qz*qz - qw*qw;
	//-180 to 180 yaw
	yaw   = (float)std::atan2( r11, r12 );
//	printf("yaw: %f\n\n",yaw);
	//-90 to 90 pitch
	pitch = (float)-std::asin( r21 );
	//-180 to 180 roll
	roll   = (float)M_PI -std::atan2( r31, r32 );
	roll = fmod(roll,2*M_PI);
	if (roll > M_PI){
		roll = roll - 2*M_PI;
	}



}

float constrain_float (float original,float lower,float upper){
	if (original> upper)
		return upper;
	else if (original<lower)
		return lower;
	else
		return original;
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

