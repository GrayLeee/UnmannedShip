#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_

#include "stdafx.h"

// 所需的一些常量定义
#define BLOCKS_PER_PACKET				12
#define VLP16_FIRINGS_PER_BLOCK			2
#define VLP16_SCANS_PER_FIRING			16
#define RAW_SCAN_SIZE					3
#define VLP16_DSR_TOFFSET				2.304f
#define VLP16_FIRING_TOFFSET			55.296f
#define VLP16_BLOCK_TDURATION			110.592f
#define DISTANCE_RESOLUTION				0.002f
#define MIN_INTENSITY					0.0f
#define MAX_INTENSITY					256.0f
#define MIN_DISTANCE					0.0f
#define MAX_DISTANCE					100.0f
const double PI = 3.14159265358979323846;

// 作为一个静态类使用
// 该类实现了将velodyne包转换为点云数据的功能
// 同时添加了图像处理的功能，可以根据强度显示图像
class PointCloud
{
public:
	~PointCloud(){}
	static void Init_OutMsg(Msg::VelodynePackets& scanMsg, Msg::outMsg& out);		// 初始化帧数据
	static void ScanToOut(Msg::VelodynePackets& scanMsg, Msg::outMsg& out);			// 转换一帧数据
	static void SetPoints(Msg::packets &pkt, Msg::outMsg& out);						// 将velodyne包转换为点云数据
	static Msg::MatArray GetMatArray(Msg::outMsg& out);								// 将点云数据映射到预处理的Mat数组里

	static Mat OutToMat(Msg::outMsg out);											// 点云数据转Mat数组
private:
	PointCloud() {}
};


#endif // !POINTCLOUD_H