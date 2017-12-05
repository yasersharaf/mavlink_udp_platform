#ifndef CLIENT_H_
#define CLIENT_H_


#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdarg.h> 
#include <sys/time.h>
#include "mavlink/v1.0/common/mavlink.h"



class Client {

    public:
        /**member fields **/
        char* server;
        int port;
        int status;
        
        //**instatsiate client
        Client(char * server, int port); 
        Client();
        ~Client(); 

        /**open and close connection**/
        void open_connection();
        void close_connection();
        
        void handle_quit( int sig );

	
	/**TODO: update mavlink stuff**/
        int read_message(mavlink_message_t& message);
        int write_message(mavlink_message_t message);
	
    private:
    	int fd;
        int connect_to_server();
	
    	void sendall(char* buffer, int* len); //low level write to socket		
    	void readall(char* buffer ,int* len); //low level read to socket
};




#endif


