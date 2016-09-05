/*
*
* Copyright (c) 2016, Yanpeng Li <lyp40293@gmail.com>.
* All rights reserved.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <string.h>
#include <unistd.h>
#include <openssl/md5.h>
#include <sstream>
#include <iostream>
#include <string>
#include "youtuface_sdk.h"

#define SERVER_PORT 19888 
#define SOCKET_QUEUE	5
#define BUFFER_SIZE	1024

using namespace std;

class YoutuServer{
public:
	int initialServer();
	void initialYoutu();
	int runServer();

private:
	int receiveFile();
	int checkMD5(char* file, char* md5);
	int identify(char* file);
	string getPersonInfo() { return _person_info; }
	int getFileInfo(char* info, char* file_name, int* file_size, char* md5);
	int _sockfd;
	struct sockaddr_in _ser_addr;
	YoutuFace _youtu;
	string _model = "data.model";
	string _photos = "photos.list";
	string _person_info;
};

void YoutuServer::initialYoutu(){
	_youtu.Init(_model);
	_youtu.RegisterFace(_photos);
}

int YoutuServer::initialServer(){
	cout << "Initial Server..." << endl;
	bzero(&_ser_addr, sizeof(_ser_addr));
	_ser_addr.sin_family = AF_INET;
	_ser_addr.sin_addr.s_addr = htons(INADDR_ANY);
	_ser_addr.sin_port = htons(SERVER_PORT);

	_sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (_sockfd < 0){
		printf("Create socket failed.\n");
		return -1;
	}

	int yes = 1;
	if (setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1){
		printf("Set Sock opt failed\n");
		return -1;
	}	

	return 0;
}

int YoutuServer::getFileInfo(char* info, char* file_name, int* f_size, char* md5){
	int i = 0, n = i;
	char file_size[128];
	bzero(file_size, 128);
	while(1){
		if (info[i] == '\0') break;
		if (info[i] == '='){
			file_name[i] = '\0';
			i++; 
			break;
		}

		file_name[i] = info[i];
		i++;
	}

	n = i;
	while(1){
		if (info[i] == '\0') break;
		if (info[i] == '='){
			file_size[i-n] = '\0';
			i++;
			break;
		}

		file_size[i-n] = info[i];
		i++;
	}

	n = i;
	while(1){
		if (info[i] == '\0' || i-n == 32){
			md5[i-n] = '\0';
			break;
		}

		md5[i-n] = info[i];
		i++;
	}

	*f_size = atoi(file_size);

	return 0;
}

int YoutuServer::checkMD5(char* file, char* md5){
	cout << "Check md5 value." << endl;
	char md5sum[33];
	MD5_CTX x;
	int bytes;
	unsigned char d[MD5_DIGEST_LENGTH];
	unsigned char buf[BUFFER_SIZE];
	MD5_Init(&x);
	FILE* fp = fopen(file, "rb");
	while( (bytes = fread(buf, 1, BUFFER_SIZE, fp)) != 0){
		MD5_Update(&x, buf, bytes);
	}
	MD5_Final(d, &x);
	int i = 0;
	for (i = 0; i < MD5_DIGEST_LENGTH; i++)
		sprintf(md5sum+(i*2), "%02x", d[i]);
	md5sum[32] = '\0';
	fclose(fp);

	if (!strcmp(md5sum, md5)){
		cout << "valid file" << endl;
		return 0;
	}
	else{
		cout << "Invalid file" << endl;
		return -1;
	}
}

int YoutuServer::receiveFile(){
	cout << "Receive image file." << endl;
	char buffer[BUFFER_SIZE];
	struct sockaddr_in client_addr;
	socklen_t len = sizeof(client_addr);

	int conn = accept(_sockfd, (struct sockaddr*)&client_addr, &len);
	if (conn < 0){
		printf("Connection failed.\n");
		return -1;
	}

	cout << "Receiving image from " << client_addr.sin_addr.s_addr << endl;
	char info[128];
	char file_name[128];
	int f_size;
	char md5[33];
	bzero(info, 128);
	bzero(file_name, 128);
	bzero(md5, 33);
	recv(conn, info, 128, 0);
	getFileInfo(info, file_name, &f_size, md5);
	cout << "File name is: " << file_name << endl;
	cout << "File size is: " << f_size << endl;
	cout << "MD5 is: " << md5 << endl;

	string msg_lock = "lock";
	string msg_unlcok = "unlock";

	send(conn, msg_lock.c_str(), msg_lock.size(), 0);

	FILE* fp = fopen(file_name, "wb");
	int rec_size = 0;
	while(rec_size < f_size){
		int write_size = 0;
		bzero(buffer, BUFFER_SIZE);
		if (f_size - rec_size < BUFFER_SIZE){
			recv(conn, buffer, BUFFER_SIZE, 0);
			write_size = f_size - rec_size;
			rec_size = f_size;

		}
		else{
			recv(conn, buffer, BUFFER_SIZE, 0);
			write_size = BUFFER_SIZE;
			rec_size += BUFFER_SIZE;
		}
		printf("Size is: %d\n", rec_size);
		fwrite(buffer, 1, write_size, fp);
		usleep(500);
	}
	fclose(fp);


	usleep(500);

	identify(file_name);
	string person_info = getPersonInfo();
	send(conn, person_info.c_str(), person_info.length(), 0); 
	// if (!checkMD5(file_name, md5)){
	// 	// transfer success.
	// 	identify(file_name);
	// 	string person_info = getPersonInfo();
	// 	send(conn, person_info.c_str(), person_info.length(), 0); 
	// }
	// else{
	// 	send(conn, "null", strlen("null"), 0); 
	// }
	usleep(500);
	send(conn, msg_unlcok.c_str(), msg_unlcok.size(), 0);
	close(conn);

	return 0;
}

int YoutuServer::identify(char* file){
	cout << "Identify image." << endl;
	cv::Mat img = cv::imread(file);
	string person;
	double score;
	_youtu.IdentifyFace(img, cv::Rect(0,0,img.cols, img.rows), person, score);
	cout << "Person id is: " << person << "," << "Score is: " << score << endl;

	ostringstream ss;
	ss << score;

	_person_info = person + '=' + ss.str();

	return 0;
}

int YoutuServer::runServer(){
	if ( bind(_sockfd, (struct sockaddr*)&_ser_addr, sizeof(_ser_addr)) == -1){
		printf("Bind socket failed.\n");
		return -1;
	}

	cout << "Listening..." << endl;
	if (listen(_sockfd, SOCKET_QUEUE) == -1){
		printf("Listen socket failed.\n");
		return -1;
	}

	while(1){
		if (receiveFile() < 0){
			break;
		}
	}
	
	close(_sockfd);
	return 0;
}

int main(void){
	YoutuServer server;
	server.initialYoutu();
	server.initialServer();
	server.runServer();
	
	return 0;
}



