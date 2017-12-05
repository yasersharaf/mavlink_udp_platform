// ESP8266 WiFi <-> UART Bridge
// by RoboRemo
// www.roboremo.com
#include <ESP8266WiFi.h>
// Disclaimer: Don't use RoboRemo for life support systems
// or any other situations where system failure may affect
// user or environmental safety.

// config: ///////////////////////////////////////

#define UART_BAUD 57600
#define packTimeout 1 // ms (if nothing more on UART, then send packet)
#define bufferSize 240


// ESP WiFi mode:

//#define MODE_AP // phone connects directly to ESP
//#define MODE_STA // ESP connects to router
#define MODE_STB // ESP connects to router

//#define PROTOCOL_TCP
#define PROTOCOL_UDP


#ifdef MODE_AP
// For AP mode:
const char *ssid = "Jawn1";  // You will connect your phone to this Access Point
const char *pw = "qwerty123"; // and this is the password
IPAddress ip(192, 168, 0, 99); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 6789; // and this port
// You must connect the phone to this AP, then:
// menu -> connect -> Internet(TCP) -> 192.168.0.1:9876
#endif


#ifdef MODE_STA
// For STATION mode:
const char *ssid = "OpenWrt";  // Your ROUTER SSID
const char *pw = ""; // and WiFi PASSWORD
const int port = 6789;
// You must connect the phone to the same router,
// Then somehow find the IP that the ESP got from router, then:
// menu -> connect -> Internet(TCP) -> [ESP_IP]:9876
#endif

#ifdef MODE_STB
// For STATION mode:
const char *ssid = "ABCD";  // Your ROUTER SSID
const char *pw = "dcds2011"; // and WiFi PASSWORD
const int port = 3456;
// You must connect the phone to the same router,
// Then somehow find the IP that the ESP got from router, then:
// menu -> connect -> Internet(TCP) -> [ESP_IP]:9876
#endif

//////////////////////////////////////////////////


#include <ESP8266WiFi.h>

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server(port);
WiFiClient client;
#endif

#ifdef PROTOCOL_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
IPAddress remoteIp;
#endif

uint8_t buf1[72];
uint8_t i1=0;

uint8_t buf2[1024];
uint8_t i2=0;



void setup() {

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(UART_BAUD);
//  Serial.write/("Hello");

  #ifdef MODE_AP 
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  #endif

  
  #ifdef MODE_STA
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  
//  WiFi.begin(ssid, pw);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  #endif

  #ifdef MODE_STB
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  #endif


  #ifdef PROTOCOL_TCP
  Serial.println("Starting TCP Server");
  server.begin(); // start TCP server 
  #endif

  #ifdef PROTOCOL_UDP
  Serial.println("Starting UDP Server");
  udp.begin(port); // start UDP server 
#endif
}


void loop() {
 #ifdef PROTOCOL_TCP
  if(!client.connected()) { // if client not connected
    client = server.available(); // wait for it to connect
    return;
  }

  // here we have a connected client

  if(client.available()) {
    while(client.available()) {
      buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
      if(i1<1023) i1++;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    // now send to UART:
    Serial.write(buf1, i1);
    i1 = 0;
  }

  if(Serial.available()) {
    while(Serial.available()) {
      buf2[i2] = (char)Serial.read(); // read char from UART
      if(i2<1023) i2++;
    }
    // now send to WiFi:
    client.write((char*)buf2, i2);
    i2 = 0;
  }
   #endif

  
  #ifdef PROTOCOL_UDP
  // if there’s data available, read a packet
  int packetSize = udp.parsePacket();
  if(packetSize>0) {
    remoteIp = udp.remoteIP(); // store the ip of the remote device
    digitalWrite(LED_BUILTIN, HIGH);
    
    udp.read(buf1, bufferSize);
    // now send to UART:
    Serial.write(buf1, packetSize);
        // now send to WiFi:  
//    Serial.print(remoteIp);
//    Serial.print(port);
//    udp.beginPacket(remoteIp, port); // remote IP and port
//    udp.write("Hi there");
//    udp.endPacket();
  }

  if(Serial.available()) {

    // read the data until pause:
    //Serial.println("sa");
    
    while(1) {
      if(Serial.available()) {
        buf2[i2] = (char)Serial.read(); // read char from UART
        if(i2<bufferSize-1) {
          i2++;
        }
      } else {
        //delayMicroseconds(packTimeoutMicros);
        //Serial.println("dl");
        delay(packTimeout);
        if(!Serial.available()) {
          //Serial.println("bk");
          break;
        }
      }
    }

    // now send to WiFi:  
    udp.beginPacket(remoteIp, port); // remote IP and port
    udp.write(buf2, i2);
    udp.endPacket();
    i2 = 0;
  }
    
#endif
  
}


