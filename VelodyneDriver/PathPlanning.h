#ifndef PATH_PALNNING_H

#include "stdafx.h"

const float EPSILON_OF_GRAVITATION = 100.0f;			//引力尺度因子
const float ETA_OF_REPULSION = 20.0f;					//斥力尺度因子
const float RHO_OF_REPULSION = 800.0f;					//障碍物影响半径
const float SIZE_OF_STEP = 10.0f;						//指点半径
const float STARTING_FORCE = 20.0f;

#define PI 3.1415926

class PathPlanning
{
public:
	//todo
	void GetGPS();
	void GPSToPixel();
	void PixelToGPS();

	static Msg::Force SelfToObs(Msg::ObstacleMoveParameter & obs, cv::Point2i curPos);					//计算当前位置受到障碍物点集的合力	
	static cv::Point2i Pointing(cv::Point2i presentPosition, Msg::Force jointForce, cv::Mat & mat);		//获取下一个目标航点位置
	static Msg::Force JointForce(vector<Msg::Force> forces);											//计算力集合的合力

private:	
	PathPlanning() = default;
	~PathPlanning() = default;

	static float GetDistance(cv::Point2i p1, cv::Point2i p2);				//计算两点间距离
	static Msg::Force GetGravitation(cv::Point2i p1, cv::Point2i p2);		//计算引力
	static Msg::Force GetRepulsion(cv::Point2i p1, cv::Point2i p2);		//计算斥力
	static Msg::Force GetJointForce(Msg::Force f1, Msg::Force f2);				//计算合力
	static Msg::Force DisturbingPower(Msg::Force f);							//合力为0时人为添加扰动
	static int GetPosPix(cv::Point2i p, cv::Mat & mat);						//获取某个像素点的像素值

	
};


#endif // !PATH_PALNNING_H

