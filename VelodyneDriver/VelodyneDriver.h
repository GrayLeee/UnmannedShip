#ifndef _VELODYNEDRIVER_H_
#define _VELODYNEDRIVER_H_

#include "stdafx.h"


// 该类实现了连接到指定UDP端口和IP地址并接收UDP包，转换为velodyne包的功能
class VelodyneDriver
{
public:
	VelodyneDriver(const string device_ip, const int UDP_port);
	~VelodyneDriver();
	int ConnectSocket();														// 连接到UDP短裤
	int ReceviceData(char* recvData, sockaddr_in& remoteAddr);					// 接收一个包的数据
	void PrintData(char* data, int len);										// 打印一个UDP包数据
	int GetPacks(Msg::VelodynePackets& v_pack, int seq);						// 接收一帧数据
	uint64_t GetSystemTime();													// 计算当前系统时间戳
	string GetDeviceIp();														// 返回设置的设备IP
	int GetUdpPort();															// 返回设备的端口号
private:
	WSADATA wsaDATA;
	WORD sockVersion;
	SOCKET serSocket;
	sockaddr_in remoteAddr;
	string dev_ip;
	int port;
	const int NPACKS = 76 * 4;
};

#endif // !VELODYNEDRIVER_H