/*
 * UDP.cpp
 *
 *  Created on: Aug 15, 2017
 *      Author: yaser
 */


#include "UDP_Client.hpp"
UDP_Client::UDP_Client(std::string str,int port_num, uint16_t id,uint64_t platform_epoch_udp_){
	strcpy(target_ip, str.c_str());
	port = port_num;
	udp_id = id;
	std::cout<<"ip:"<<target_ip<<"\t port:"<<port<<std::endl;
	platform_epoch_udp =platform_epoch_udp_;
	status = PORT_CLOSED;
}

UDP_Client::~UDP_Client(){
	disconnect();

}

int UDP_Client::get_sock(){
	return sock;
}

uint64_t UDP_Client::microsSinceEpoch()
{

	struct timeval tv;

	uint64_t micros = 0;

	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000L + (uint64_t)tv.tv_usec;

	return micros;
}

void UDP_Client::connect(){

	printf("client %d connecing ...",udp_id);

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(port);
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(port);
	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);	/* Bind the socket to port 6789 - necessary to receive packets from clients */
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
	{
		perror("error bind failed");
		status = PORT_ERROR;
		close(sock);
		exit(EXIT_FAILURE);
	}
	/* Attempt to make it non blocking */

	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)

	{
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		status = PORT_ERROR;
		close(sock);
		exit(EXIT_FAILURE);
	}

	status = PORT_OPEN;


}

uint16_t UDP_Client:: write_message(const mavlink_message_t &msg){
//	float position[6] = {};
	uint8_t buf[buffer_length];
//	std::cout<<"buffer length:"<<buffer_length<<std::endl;
	int bytes_sent;
//	mavlink_message_t msg;
	uint16_t len;
//	int i = 0;
//	mavlink_msg_vicon_position_estimate_pack(1,200,&msg,microsSinceEpoch(),
//			position[0], position[1], position[2],
//			position[3], position[4], position[5]);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
	memset(buf, 0, buffer_length);
	printf("Sent!\n");
	return len;

}

ssize_t UDP_Client::read_message(mavlink_message_t &msg){
	//struct sockaddr_in fromAddr;
	uint8_t buf[buffer_length];
	ssize_t recsize;
	socklen_t fromlen;

	unsigned int temp = 0;
	recsize = recvfrom(sock, (void *)buf, buffer_length, 0, (struct sockaddr *)&gcAddr, &fromlen);
	if (recsize > 0)
	{
		// Something received - print out all bytes and parse packet
		mavlink_status_t status;
//		mavlink_message_t msg;

		printf("Bytes Received: %d\n", (int)recsize);
		for (int i = 0; i < recsize; ++i)
		{
			temp = buf[i];
//			printf("%02x ", (unsigned char)temp);
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
			{
				// Packet received
//				printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

				switch(msg.msgid) {
				//** only one case for now
					case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
					{
						mavlink_local_position_ned_t ned_message;
						mavlink_msg_local_position_ned_decode(&msg, &ned_message);

//						printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");

						uint64_t rcpt_time = microsSinceEpoch();

//						printf("%llu\n",ned_message.time_boot_ms);
						printf("%llu\n",rcpt_time);
						break;
					}
					case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
					{
						mavlink_vicon_position_estimate_t vicon_message;
						mavlink_msg_vicon_position_estimate_decode(&msg, &vicon_message);

//						printf("MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE\n");

						uint64_t rcpt_time = microsSinceEpoch();

//						printf("%ld\n",vicon_message.usec);
//						printf("%llu\n",rcpt_time);
						printf("%s\t\t\t\t\t\t\t\t\t\t\t\t\t\t delay %u = %llu\n%s",KMAG,udp_id,rcpt_time-vicon_message.usec,KNRM);

						break;
					}
					case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
					{
						mavlink_hil_rc_inputs_raw_t optitrack_message;
						mavlink_msg_hil_rc_inputs_raw_decode(&msg, &optitrack_message);

//						printf("MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE\n");

						uint64_t rcpt_time = microsSinceEpoch();

//						printf("%ld\n",vicon_message.usec);
//						printf("%llu\n",rcpt_time);
						printf("%s\t\t\t\t\t\t\t\t\t\t\t\t\t\t delay %u = %llu, %llu, %llu\n%s",KBLU,udp_id,rcpt_time-optitrack_message.time_usec,rcpt_time,optitrack_message.time_usec,KNRM);

						break;
					}
				}
			}
		}
		printf("\n");
	}

	memset(buf, 0, buffer_length);
	return recsize;
}

void UDP_Client::disconnect(){

    try {
        close(sock);
    	status = PORT_CLOSED;
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not close socket\n");
    }



}

void UDP_Client::handle_quit(int sig) {

    try {
        disconnect();
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not close udp port\n");
    }



}
