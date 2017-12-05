#include "client.h"
#define PORT_OPEN   1;
#define PORT_CLOSED 0;
#define PORT_ERROR -1;



Client::Client(char * server_name , int portno) {

     fprintf(stdout,"client made\n");
     server = server_name;
     port = portno;
     fd = -1; //set fd to -1 as a default
}

Client::Client() {}
Client::~Client() {}

void Client::handle_quit(int sig) {

    try {
        close_connection();
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop serial port\n");
    }



}

/**returns a file descriptor to write to**/
int Client::connect_to_server() {

    int clientfd;
    struct hostent *hp;
    struct sockaddr_in serveraddr;
    char errbuf[256];                                   /* for errors */

    /* create a socket */
    if ((clientfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        fprintf(stderr, "Error creating socket\n");
        exit(1);
    }
        
    /* Fill in the server's IP address and port */ 
    if ((hp = gethostbyname(server)) == NULL) {
        sprintf(errbuf, "%d", h_errno);
        fprintf(stderr,"DNS error: DNS error ");
        exit(1);
   }
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)hp->h_addr_list[0],
          (char *)&serveraddr.sin_addr.s_addr, hp->h_length);
    
    serveraddr.sin_port = htons(port);

    /* connect */
    if (connect(clientfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0) {
        fprintf(stderr, "Error connecting\n");
        exit(1);
    }	
    return clientfd;
}


void Client::open_connection() {

	
	if((fd =  connect_to_server()) == -1){

		fprintf(stderr, "Error binding to port\n");
		exit(0); //exit gracefully
        status = PORT_ERROR; 

	}	else  {
        status = PORT_OPEN;
    }

	fprintf(stdout, "Opened Connection\n");


}



void Client::close_connection() {

	close(fd); //close file descriptor
    status = PORT_CLOSED;

}


void Client::sendall(char * buffer, int* len) {

	if (fd == -1) {
		fprintf(stderr, "port is not open"); 
		return; 
		
	}
	int  nremain = *len;
	int  nsofar = 0; 
	char* buffer_cp = buffer;
	while(nremain > 0) {

		if((nsofar = write(fd ,buffer_cp, nremain)) <=0){  //TODO look at send vs write
			nsofar = 0; //once writing stream is done, set to 0 

		}

		buffer_cp+=nsofar; //move to transmitting next byte
		nremain  -= nsofar; //update how many bytes we have sent out
	}

}

void Client::readall(char* buffer, int* len) {



	char* buffer_cp = buffer;
	ssize_t nsofar = 1;
	size_t nremain = *len;
	
	while(nsofar == 0) {

		if((nsofar =  read(fd, buffer_cp, nremain)) < 0) {



        }
        buffer_cp += nsofar;
        nremain -= nsofar;


    }


}



	

int Client::read_message(mavlink_message_t &message) {


    //set up previous 
    mavlink_status_t lastStatus;
    lastStatus.packet_rx_drop_count = 0; //set drop count to 0


    uint8_t cp; 
    mavlink_status_t status;
    int8_t msg_rec = false;


    if ((msg_rec = recv(fd, (char*)&cp,1, 0)) == -1) {
        fprintf (stderr, "fail to recv\n");
    } else {
       // fprintf (stderr, "Its working\n\n\n");
    }

    if(msg_rec > 0) {

         //******* PARSING THE MESSAGE ******//
        msg_rec = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status); 


        if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)  )
        {
//            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
            unsigned char v=cp;
//            fprintf(stderr,"%02x ", v);
        }
        lastStatus = status;
    

    } else {

        fprintf(stderr, "Could not read from TCP\n" );
        return -1;


    }




    
    return msg_rec;   //return length





    
}

int Client::write_message(mavlink_message_t message) {
	
    if (fd == -1) {

        fprintf(stderr, "Port is not Open!\n");
        exit(0);

    }
    char message_buffer[300];


    //convert message into a buffer
    int len =  mavlink_msg_to_send_buffer((uint8_t*)message_buffer, &message);


//	fprintf(stdout, "Messages are being sent\n"); //log message
//    fprintf(stdout, "=============================\n\n"); //log message TODO: change to stderr
	
//    fprintf(stdout, "%s\n\n",message_buffer); //log message TODO: change to stderr
    sendall(message_buffer, &len); //send message


	
//	fprintf(stdout, "Messages have been sent\n\n");   //log message

    return len;
}









