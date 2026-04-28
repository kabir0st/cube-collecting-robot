#include <wiringPiSPI.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

using namespace std;


int Send_Flag = 0;
int Global_Counter;

void *Server(void *){

	int Received_Bytes;
	char recv_buffer[100];
	char Send_Buffer[32];
     int sockfd, newsockfd;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        puts("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(12345);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              puts("ERROR on binding");
     listen(sockfd,1);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          puts("ERROR on accept");
     else
        puts("Client connected!");


	while(1){
		Received_Bytes = recv(newsockfd, recv_buffer, 100, 0);
		cout << "Server received " << Received_Bytes << " bytes and msg = "<< recv_buffer <<endl ;
		sprintf(Send_Buffer,"%d",Global_Counter+1);
		send( newsockfd, Send_Buffer, 10 ,0);
	}
	close(newsockfd);
    close(sockfd);
}

void *Client(void *){
	
	int Received_Bytes;
	char buff[100];
	int sockfd, portno = 12345, n;
    char serverIp[] = "127.0.0.1";
    struct sockaddr_in serv_addr;
    struct hostent *server;

    if ( ( sockfd = socket(AF_INET, SOCK_STREAM, 0) ) < 0 )
        puts( "ERROR opening socket");

    if ( ( server = gethostbyname( serverIp ) ) == NULL ) 
        puts("ERROR, no such host\n") ;
    
    bzero( (char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy( (char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    if ( connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) 
        puts( "ERROR connecting");

	
	while(1){
		if(Send_Flag){
			Send_Flag = 0;
			sprintf(buff,"%d",Global_Counter);
			send( sockfd, buff, 10 ,0);
			Received_Bytes = recv(sockfd, buff, 100, 0);
			Global_Counter = buff[0] - '0';
			cout << "client received " << Received_Bytes << " bytes and msg = "<< buff <<endl ;
		}
	}
}
