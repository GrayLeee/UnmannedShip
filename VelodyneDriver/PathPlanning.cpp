#include "stdafx.h"
#include "PathPlanning.h"
#include "Preprocess.h"

//计算两点间距离
float PathPlanning::GetDistance(cv::Point2i p1, cv::Point2i p2)
{
	return float(sqrt(
		pow(p1.x - p2.x, 2) + 
		pow(p1.y - p2.y, 2)
	));
}

//计算引力
Msg::Force PathPlanning::GetGravitation(cv::Point2i p1, cv::Point2i p2)
{
	Msg::Force grav;
	grav.value = EPSILON_OF_GRAVITATION * GetDistance(p1, p2) + STARTING_FORCE;
	grav.theta = atan2f(p2.y - p1.y, p2.x - p1.x);
	return grav;
}

// 计算斥力
Msg::Force PathPlanning::GetRepulsion(cv::Point2i p1, cv::Point2i p2)
{
	Msg::Force Repulsion;
	if (GetDistance(p1, p2) > RHO_OF_REPULSION)
	{
		Repulsion.value = 0;
	}
	else
	{
		Repulsion.value = ETA_OF_REPULSION * (
			(RHO_OF_REPULSION - GetDistance(p1, p2)) / 
			(RHO_OF_REPULSION * pow(GetDistance(p1, p2), 3)));
		Repulsion.theta = atan2f(p1.y - p2.y,p1.x - p2.x);
	}
	return Repulsion;
}

//计算合力
Msg::Force PathPlanning::GetJointForce(Msg::Force f1, Msg::Force f2)
{
	Msg::Force JointForce;
	JointForce.value = sqrt(
		pow(f1.value * sinf(f1.theta) + f2.value * sinf(f2.theta), 2) + 
		pow(f1.value * cosf(f1.theta) + f2.value * cosf(f2.theta), 2));
	JointForce.theta = atan2f(
		f1.value*sinf(f1.theta) + f2.value*sinf(f2.theta), 
		f1.value*cosf(f1.theta) + f2.value*cosf(f2.theta));
	return JointForce;
}

//人为扰动力
Msg::Force PathPlanning::DisturbingPower(Msg::Force f)
{
	if (f.value == 0)
	{
		f.theta = float((rand() % 360) * PI / 180);
		f.value = 10;
	}
	return f;
}

//获取下一个目标航点
cv::Point2i PathPlanning::Pointing(cv::Point2i presentPosition, Msg::Force jointForce, cv::Mat & mat)
{
	cv::Point2i nextPoint;
	nextPoint.x = presentPosition.x + SIZE_OF_STEP * cosf(jointForce.theta);
	nextPoint.y = presentPosition.y + SIZE_OF_STEP * sinf(jointForce.theta);

	int backPix = Preprocess::getBackGroundPix(mat);
	
	//如果指点有障碍物，则逆时针旋转15度，直至指的点无障碍物
	while (GetPosPix(nextPoint, mat) != backPix)
	{
		nextPoint.x = presentPosition.x + SIZE_OF_STEP * cosf(jointForce.theta + PI * 15 / 180.0);
		nextPoint.y = presentPosition.y + SIZE_OF_STEP * sinf(jointForce.theta + PI * 15 / 180.0);
	}
 
	return nextPoint;
}

//计算力集合的合力
Msg::Force PathPlanning::JointForce(vector<Msg::Force> forces)
{
	while (forces.size() > 1)
	{
		Msg::Force f1 = forces.back();
		forces.pop_back();
		Msg::Force f2 = forces.back();
		forces.pop_back();
		Msg::Force curJointForce = GetJointForce(f1, f2);
		forces.push_back(curJointForce);
	}
	return forces.front();
}

//获取指定点的像素值
int PathPlanning::GetPosPix(cv::Point2i p, cv::Mat & mat)
{
	// 坐标轴不同
	return mat.at<int>(p.y, p.x);
}

//计算当前位置与某个障碍物之间的合力
Msg::Force PathPlanning::SelfToObs(Msg::ObstacleMoveParameter & obs, cv::Point2i curPos)
{
	vector<Msg::Force> forces;
	for (int i = 0; i < obs.pixID.size(); i++) 
	{
		cv::Point2i curPoint = Preprocess::pixIDToCoordinate(obs.pixID[i], 80, 80);
		Msg::Force curForce = GetRepulsion(curPos,curPoint);
		forces.push_back(curForce);
	}

	return JointForce(forces);
}






