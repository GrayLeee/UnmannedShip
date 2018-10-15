#ifndef _STDAFX_H_
#define _STDAFX_H_


// 这里的引用顺序不要变
#include <WinSock2.h>
#include <Windows.h>
#include <cstdio>
#include <tchar.h>

// 头文件引用部分
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sys/timeb.h>
#include <WS2tcpip.h>
#include <string>
#include <vector>
#include <sstream>
using namespace cv;
using namespace std;
#pragma comment(lib, "ws2_32.lib")


// 消息定义部分
namespace Msg
{
	// 标准头，包括序列号和时间戳
	struct header
	{
		int seq;
		time_t stamp;
	};

	// UDP包格式
	struct packets
	{
		time_t stamp;
		char data[1206];
	};

	// 一帧雷达数据格式
	struct VelodynePackets
	{
		header head;
		vector<packets> packs;
	};
 
	struct fields
	{
		string name;
		int offset;
		const int count = 1;
	};

	// 点格式
	struct point
	{
		float x;
		float y;
		float z;
		float intensity;
	};

	// 一帧转换后的点云数据
	struct outMsg
	{
		header head;
		const int height = 1;
		int width;
		fields field[4];
		vector<point> points;
		bool is_bigendian = false;
		const int point_step = 16;
		int row_step;
		bool is_dense;
	};

	union two_byte
	{
		uint16_t uint;
		uint8_t byte[2];
	};


	// 前置数组，保存了投影到图片内的点云数据
	struct MatArray
	{
		int** matrix;
		int** count;
		int row;
		int column;
		void Init(float length, float width, float unit)
		{
			row = int(width / unit);
			column = int(length / unit);

			matrix = (int **)malloc(row * sizeof(int *));
			count = (int **)malloc(row * sizeof(int *));
			for (int i = 0; i < row; i++)
			{
				matrix[i] = (int *)malloc(column * sizeof(int));
				count[i] = (int *)malloc(column * sizeof(int));
			}
			// 初始化
			for (int i = 0; i < row; i++)
			{
				for (int j = 0; j < column; j++)
				{
					matrix[i][j] = 0;
					count[i][j] = 0;
				}
			}
		}

		void Destory()
		{
			for (int i = 0; i < row; i++)
			{
				free(matrix[i]);
				free(count[i]);
			}
			free(matrix);
			free(count);
		}
	};

	// 障碍物定义
	struct ObstacleMoveParameter
	{
		time_t time;
		int obstacleID;	
		int pix;														
		int pix_nums;			
		vector<int> pixID;
		cv::Point2d barycentre;														
		int appear_counts;															
		bool is_obstacle;															

		float boatLength;															
		float speed;																
		float accelerated_speed;													
		float angle;																
	};

	// 作用力类型定义
	struct Force
	{
		float value;			//力的大小
		float theta;			//力相较于正北方向沿顺时针的偏转角
	};
}

#endif // !STDAFX_H