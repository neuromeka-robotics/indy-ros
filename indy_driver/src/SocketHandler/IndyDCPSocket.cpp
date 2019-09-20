#include "IndyDCPSocket.h"

IndyDCPSocket::IndyDCPSocket(void)
: _thread(NULL)
, _isQuit(false)
, _sockFd(-1)
, _invokeID(0)
{
}

IndyDCPSocket::~IndyDCPSocket(void)
{
	if (_sockFd >= 0)
	{
		close(_sockFd);
		_sockFd = -1;
	}
	printf("IndyDCPSocket: Terminated\n");
}

bool IndyDCPSocket::init(std::string robotName, std::string ip, int port)
{
	_robotName = robotName;
	_ip = ip;
	_port = port;
	_thread = new std::thread(&IndyDCPSocket::run, this);

	return true;
}

bool IndyDCPSocket::isWorking()
{ 	
	return (!_isQuit);
}

void IndyDCPSocket::stop() 
{ 
	_isQuit = true;
	_thread->join();
	delete _thread;
}

bool IndyDCPSocket::sendCommand(const int cmdID, const Data & data, const unsigned int len)
{
	if (_sockFd < 0)
		return false;

	HeaderCommand header;
	strcpy(header.val.robotName, _robotName.c_str());
	header.val.robotVersion[0] = '\0';
	header.val.stepInfo = 0x02;
	header.val.sof = 0x34;
	header.val.invokeId = _invokeID++;
	header.val.cmdId = cmdID;
	header.val.dataSize = len;

	unsigned char writeBuff[1024];
	memcpy(writeBuff, header.byte, SIZE_HEADER_COMMAND);
	if (send(_sockFd, writeBuff, SIZE_HEADER_COMMAND, 0) == -1)
	{
		printf("IndyDCPSocket: Can't send header (errno=%d)\n", errno);
		close(_sockFd);
		_sockFd = -1;
		return false;
	}

	if (header.val.dataSize > 0)
	{
		memcpy(writeBuff, &data, header.val.dataSize);
		if (send(_sockFd, writeBuff, header.val.dataSize, 0) == -1)
		{
			printf("IndyDCPSocket: Can't send data (errno=%d)\n", errno);
			close(_sockFd);
			_sockFd = -1;
			return false;
		}
//		printf("IndyDCPSocket: Send Data (%d bytes)!\n", header.val.dataSize);
	}

	return true;
}

bool IndyDCPSocket::sendExData(unsigned char const * const exData, const unsigned int len)
{
	if (send(_sockFd, exData, len, 0) == -1)
	{
		printf("IndyDCPSocket: Can't send extended data (errno=%d)\n", errno);
		close(_sockFd);
		_sockFd = -1;
		return false;
	}

	return true;
}


bool IndyDCPSocket::getFeedback(const int cmdID, Data & data, unsigned int & dataLen)
{
	if (_sockFd < 0)
		return false;

	HeaderCommand resHeader;
	Data resData;
	unsigned char readBuff[1024];

	//Recv
	int len, cur;

	len = 0;
	cur = 0;
	while (true)	//read header
	{
		len = recv(_sockFd, readBuff+cur, SIZE_HEADER_COMMAND-cur, MSG_WAITALL);
		if (len == 0) usleep(1e5);
		else if (len > 0)
		{
			cur += len;
			if (cur == SIZE_HEADER_COMMAND) break;
		}
		else
		{
			printf("IndyDCPSocket : Socket can't receive header (errno=%d)\n", errno);
			close(_sockFd);
			_sockFd = -1;
			break;
		}
	}
	if (_sockFd == -1) return false;
	memcpy(resHeader.byte, readBuff, SIZE_HEADER_COMMAND);
//	printf("IndyDCPSocket : Read response header, dataSize=%d\n", resHeader.val.dataSize);

	if (resHeader.val.dataSize > 0)	//read data
	{
		len = 0;
		cur = 0;
		while (true)
		{
			len = recv(_sockFd, readBuff+cur, resHeader.val.dataSize-cur, MSG_WAITALL);
			if (len == 0) usleep(1e5);
			else if (len > 0)
			{
				cur += len;
				if (cur == resHeader.val.dataSize) break;
			}
			else
			{
				printf("IndyDCPSocket : Socket can't receive data (errno=%d)\n", errno);
				close(_sockFd);
				_sockFd = -1;
				break;
			}
		}

		if (_sockFd == -1) return false;
		memcpy(resData.byte, readBuff, resHeader.val.dataSize);
//		printf("IndyDCPSocket : Read response data\n");
	}

	if (resHeader.val.cmdId == cmdID && resHeader.val.sof == 0x12)
	{
		data = resData;
		dataLen = resHeader.val.dataSize;

		return true;
	}
	else
		return false;

}

void IndyDCPSocket::run(void)
{
	char inchar;
	int len, cur;
	while (!_isQuit)
	{
		if (_sockFd >= 0)
		{
			sleep(100);
		}
		else if (!_isQuit)
		{
			struct sockaddr_in server_addr;
			_sockFd = socket(AF_INET, SOCK_STREAM, 0);
			if (_sockFd < 0) {
				printf("IndyDCPSocket : Error opening socket\n");
			}

			server_addr.sin_addr.s_addr = inet_addr(_ip.c_str());
			server_addr.sin_family = AF_INET;
			server_addr.sin_port = htons(_port);

			if (connect(_sockFd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) 
			{
				printf("IndyDCPSocket : Socket connection failed.\n");
				close(_sockFd);
				_sockFd = -1;
				sleep(100);
			}
			else
			{
				printf("IndyDCPSocket : Socket is connected\n");
				continue;
			}
		}
	}

	close(_sockFd);
	_sockFd = -1;
}
