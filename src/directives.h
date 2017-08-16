/*
 * directives.h
 *
 *  Created on: Aug 15, 2017
 *      Author: yaser
 */

#ifndef DIRECTIVES_H_
#define DIRECTIVES_H_

# define M_PI  3.14159265358979323846
//# define APM_IP1 "192.168.1.113"
//# define APM_IP2 "192.168.1.115"

# define APM_IP1 "192.168.0.100"
# define APM_IP2 "192.168.0.102"

# define APM_PORT1 6789
# define APM_PORT2 7890



#define usingUDP //If using UDP (toward multidrone)
//#define usingTCP //If using TCP
//#define usingSerial //If using Serial



#define PORT_OPEN   1;
#define PORT_CLOSED 0;
#define PORT_ERROR -1;




#endif /* DIRECTIVES_H_ */
