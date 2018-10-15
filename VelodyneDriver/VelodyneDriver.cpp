#include "stdafx.h"
#include "VelodyneDriver.h"

VelodyneDriver::VelodyneDriver(const string device_ip, const int UDP_port) : dev_ip(device_ip), port(UDP_port)
{
	sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaDATA) != 0)
		printf("Set IP version failed!\n");
	if (ConnectSocket())
		printf("Connect error!\n");
	else
		printf("Connect successful!\n");
}

VelodyneDriver::~VelodyneDriver()
{
	closesocket(serSocket);
	WSACleanup();
}

int VelodyneDriver::ConnectSocket()
{
	serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (serSocket == INVALID_SOCKET)
	{
		printf("socket error!\n");
		return 1;
	}
	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(port);
	serAddr.sin_addr.S_un.S_addr = INADDR_ANY;
	// 为了解决std的bind和socket的bind冲突
	// 使用限定符std::bind()和::bind()
	if(::bind(serSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{
		printf("Bind error!\n");
		return 1;
	}
	return 0;
}

// 接收一个UDP包数据
int VelodyneDriver::ReceviceData(char * recvData, sockaddr_in & remoteAddr)
{
	int nAddrLen = sizeof(remoteAddr);
	memset(recvData, '0', sizeof(recvData));
	int rc = recvfrom(serSocket, recvData, 1206, 0, (sockaddr *)&remoteAddr, &nAddrLen);
	if (rc < 0)
	{
		printf("received failed!\n");
		return -1;
	}
	return rc;
}

void VelodyneDriver::PrintData(char * data, int len)
{
	for (int i = 0; i < len; i++)
	{
		if (i > 0)
			printf(", ");
		printf("%d", (int)data[i]);
		if (i % 10 == 0 && i > 0)
			printf("\n");
	}
	printf("\n");
}

// 接收一帧数据
int VelodyneDriver::GetPacks(Msg::VelodynePackets & v_pack, int seq)
{
	v_pack.head.seq = seq;
	v_pack.head.stamp = GetSystemTime();

	for (int i = 0; i < NPACKS; i++)
	{
		Msg::packets pkt;
		pkt.stamp = v_pack.head.stamp;
		int rc = ReceviceData(pkt.data, remoteAddr);
		if (inet_ntoa(remoteAddr.sin_addr) != dev_ip)
		{
			printf("Packet not from Velodyne Lidar, it from %s\n", inet_ntoa(remoteAddr.sin_addr));
			return -1;
		}
		v_pack.packs.push_back(pkt);
	}
	printf("packets count of v_pack is %zd\n", v_pack.packs.size());
	return 0;
}

uint64_t VelodyneDriver::GetSystemTime()
{
	timeb t;
	ftime(&t);
	return t.time * 1000 + t.millitm;
	return uint64_t();
}

string VelodyneDriver::GetDeviceIp()
{
	return dev_ip;
}

int VelodyneDriver::GetUdpPort()
{
	return port;
}


