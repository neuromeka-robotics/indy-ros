/*
 * SocketDefine.h
 *
 *  Created on: Jul 6, 2018
 *      Author: ThachDo-MAC
 */

#ifndef SOCKETDEFINE_H_
#define SOCKETDEFINE_H_

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 6066

enum
{
	SIZE_HEADER = 52,
	SIZE_COMMAND = 4,
	SIZE_HEADER_COMMAND = 56,
	SIZE_DATA_MAX = 200,
	SIZE_DATA_ASCII_MAX = 32
};

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
struct HeaderCommandStruct
{
	char robotName[20];
	char robotVersion[12];
	unsigned char stepInfo;
	unsigned char sof;		//source of Frame
	int invokeId;
	int dataSize;
	char reserved[10];
	int cmdId;
};
#pragma pack(pop)   /* restore original alignment from stack */

union HeaderCommand
{
	unsigned char byte[SIZE_HEADER_COMMAND];
	HeaderCommandStruct val;
};

union Data
{
	unsigned char byte[SIZE_DATA_MAX];
	char asciiStr[SIZE_DATA_ASCII_MAX+1];
	char str[200];
	char charVal;
	bool boolVal;
	short shortVal;
	int intVal;
	float floatVal;
	double doubleVal;
	char char2dArr[2];
	char char3dArr[3];
	char char6dArr[6];
	char char7dArr[200];
	char charArr[200];
	int int2dArr[2];
	int int3dArr[3];
	int int6dArr[6];
	int int7dArr[6];
	int intArr[50];
	float float3dArr[3];
	float float6dArr[6];
	float float7dArr[7];
	float floatArr[50];
	double double3dArr[3];
	double double6dArr[6];
	double double7dArr[6];
	double doubleArr[25];
};

#endif /* SOCKETDEFINE_H_ */
