#ifndef _PREPROCESS_H_
#define _PREPROCESS_H_

#include "stdafx.h"

//作为静态类来使用
class Preprocess
{
public:
	static cv::Vec3b RandomColor(int value);											//生成随机颜色函数
	static void getSuspectObstacleQueue(cv::Mat marks, 
		vector<Msg::ObstacleMoveParameter> &SuspectObstacleQueue);						//得到疑是障碍物队列
	static void getActualObstacleQueue(vector<Msg::ObstacleMoveParameter> &SuspectObstacleQueue, 
		vector<Msg::ObstacleMoveParameter> &ActualObstacleQueue);						//得到确认障碍物队列
	static cv::Mat Preprocessing(cv::Mat image);										//图像预处理
	static cv::Point2i pixIDToCoordinate(int seq_pix, const int width, const int height);
	static int coordinateToPixID(cv::Point2i pix_coordinate, const int width, const int height);
	static cv::Point2d findBaryCentreOfImage(vector<int> pixID, int pix_nums, const int width, const int height);
	static int getBackGroundPix(cv::Mat image);											//得到分割后图像的背景像素值
private:
	static const int GAP_DIS = 2;
	static const int GAP_PIX = 40;
	static const int INVALID_PIX = -100;
	static const int PIX_RANGE = 256;
	Preprocess();
};

#endif