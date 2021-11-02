
// Client side C/C++ program to demonstrate Socket programming 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <tinyxml.h>
#define PORT 54603 
   
int main(int argc, char const *argv[]){ 
    int sock = 0, valread; 
    struct sockaddr_in serv_addr; 
    char buffer[1024] = {0}; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("Socket creation error"); 
        return -1; 
    } 
   
    memset(&serv_addr, '0', sizeof(serv_addr)); 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)  
    { 
        printf("Invalid address/ Address not supported"); 
        return -1; 
    } 
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("Connection Failed"); 
        return -1; 
    } 

    TiXmlDocument xml_in;
    double positions[6];
    while(1){
        valread = read( sock , buffer, 1024);
        xml_in.Parse(buffer);
        printf("%s \n",buffer ); 
        xml_in.FirstChildElement("RobotCommand") -> FirstChildElement("Position") -> QueryDoubleAttribute("X",&positions[1]);
        printf("%lf \n", positions[1]);
        memset(buffer, 0, sizeof(buffer));
        xml_in.Clear();
    }
    return 0; 
} 
