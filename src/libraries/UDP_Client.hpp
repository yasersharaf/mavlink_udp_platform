/*
 * UDP.hpp
 *
 *  Created on: Aug 15, 2017
 *      Author: yaser
 */

#ifndef UDP_CLIENT_HPP_
#define UDP_CLIENT_HPP_

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#include <string>
#include <iostream>
#include "../directives.h"


/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "mavlink/v1.0/common/mavlink.h"

using namespace std;




class UDP_Client{
public:
	UDP_Client();
	UDP_Client(std::string,int,uint16_t,uint64_t);
	~UDP_Client();
	int get_sock();
	uint64_t microsSinceEpoch();
	uint16_t write_message(const mavlink_message_t &message);
	ssize_t read_message(mavlink_message_t &message);
	void connect();
	void disconnect();
	void handle_quit(int sig);
	int status;
	int  port;
	private:

	char target_ip[15];

	struct sockaddr_in gcAddr;
	struct sockaddr_in locAddr;
	int sock;
	static const unsigned int buffer_length = 300;
	uint16_t udp_id;
	uint64_t platform_epoch_udp;

};




#endif /* UDP_HPP_ */
