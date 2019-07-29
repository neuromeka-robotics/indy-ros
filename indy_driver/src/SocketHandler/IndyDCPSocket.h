
#ifndef INDYDCPSOCKET_H_
#define INDYDCPSOCKET_H_

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <thread>

#include "SocketDefine.h"

class IndyDCPSocket
{
public:
	IndyDCPSocket();
	~IndyDCPSocket();

	void stop();

	bool init(std::string robotName, std::string ip, int port);
	bool isWorking();

	bool sendCommand(const int cmdID, const Data & data, const unsigned int len);
	bool sendExData(unsigned char const * const exData, const unsigned int len);

	bool getFeedback(const int cmdID, Data & data, unsigned int & len);

	virtual void run(void);

private:
	std::thread * _thread;
	bool _isQuit;
	int _sockFd;
	int _invokeID;
	int _cmdID;
	int _dataSize;
	int _port;

	std::string _robotName;
	std::string _ip;

	HeaderCommand header;
	Data data;
	HeaderCommand resHeader;
	Data resData;

	unsigned char _readBuff[1024];
	unsigned char _writeBuff[1024];
	double _q[6];
};

#endif /* INDYDCPSOCKET_H_ */
