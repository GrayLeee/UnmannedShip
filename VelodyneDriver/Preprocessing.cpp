#include "stdafx.h"
#include "BarrierCenter.h"

const int GAP_DIS = 2;
const int GAP_PIX = 40;
const int INVALID_PIX = -100;
const int PIX_RANGE = 256;

struct ObstacleMoveParameter
{
	int obstacleID;																//该障碍物（或疑是）编号
	int pix;																	//该障碍物（或疑是）的像素值
	int pix_nums;																//该障碍物（或疑是）所占像素个数
	vector<int> pixID;															//该障碍物（或疑是）所占的像素编号
	cv::Point2d barycentre;														//质心
	int appear_counts;															//该障碍物（或疑是）出现次数,为3则确定为障碍物
	bool is_obstacle;															//确认为障碍物

	float boatLength;															//障碍物长度
	float speed;																//障碍物速度
	float accelerated_speed;													//加速度
	float angle;																//角度
};

vector<ObstacleMoveParameter> SuspectObstacleQueue;								//疑是障碍物队列
vector<ObstacleMoveParameter> ActualObstacleQueue;								//已确认障碍物队列

cv::Vec3b RandomColor(int value);												//生成随机颜色函数
void getSuspectObstacleQueue(cv::Mat marks);									//得到疑是障碍物队列
void getActualObstacleQueue();													//得到确认障碍物队列


//该函数做：滤波，分割，标记障碍物，生成疑是障碍物队列																				
void Preprocessing(cv::Mat image)
{	
	cv::Mat imageGray;	
	cv::imshow("Gray Image", imageGray);

	
	cv::GaussianBlur(imageGray, imageGray, cv::Size(5, 5), 2);					//采用高斯滤波，按距离对邻域像素点加权平均
	cv::imshow("after filter", imageGray);

	
	cv::Canny(imageGray, imageGray, 80, 150);									//Canny边缘检测

	
	vector<vector<cv::Point>> contours;											//保存边界轮廓点

	vector<cv::Vec4i> hierarchy;												//定义层级

	cv::findContours(imageGray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());		//计算边界坐标
	cv::Mat imageContours = cv::Mat::zeros(image.size(), CV_8UC1);												//保存轮廓图

	cv::Mat marks(image.size(), CV_32S);										//Opencv分水岭第二个矩阵参数
	marks = cv::Scalar::all(0);
	int index = 0;
	int compCount = 0;

	//根据轮廓数给轮廓编号
	for (; index >= 0; index = hierarchy[index][0], compCount++)				
	{
		//对marks进行标记，对不同区域的轮廓进行编号，相当于设置注水点，有多少轮廓，就有多少注水点	
		cv::drawContours(marks, contours, index, cv::Scalar::all(compCount + 1), 1, 8, hierarchy);
		cv::drawContours(imageContours, contours, index, cv::Scalar(255), 1, 8, hierarchy);

	}

	//编号后的矩阵图像
	cv::Mat marksShows;
	cv::convertScaleAbs(marks, marksShows);
	cv::imshow("marks before Watershed", marksShows);	

	cv::watershed(image, marks);


	//分水岭算法后的矩阵图像
	cv::Mat afterWatershed;
	convertScaleAbs(marks, afterWatershed);
	cv::imshow("After Watershed", afterWatershed);

	//对可能是障碍物的区域进行颜色填充	
	cv::Mat PerspectiveImage = cv::Mat::zeros(image.size(), CV_8UC3);
	for (int i = 0; i < marks.rows; i++)
	{
		for (int j = 0; j < marks.cols; j++)
		{
			int pix = marks.at<int>(i, j);
			if (marks.at<int>(i, j) == -1)
				PerspectiveImage.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
			else
				PerspectiveImage.at<cv::Vec3b>(i, j) = RandomColor(pix);
		}
	}
	cv::imshow("After ColorFill", PerspectiveImage);

	//根据分割后的图像得到怀疑队列
	getSuspectObstacleQueue(marks);
}


//更新怀疑队列
void getSuspectObstacleQueue(cv::Mat marks)
{
	//初始化一帧内的障碍物数组
	struct ObstacleMoveParameter * Ob = (struct ObstacleMoveParameter *)malloc(PIX_RANGE * sizeof(struct ObstacleMoveParameter));
	for (int i = 0; i < PIX_RANGE; i++)
	{
		Ob[i].obstacleID = 0;										
		Ob[i].pix = INVALID_PIX;
		Ob[i].pix_nums = 0;
		Ob[i].appear_counts = 1;
		Ob[i].is_obstacle = false;
		Ob[i].boatLength = 0;
		Ob[i].speed = 0;
		Ob[i].accelerated_speed = 0;
		Ob[i].angle = 0;
	}

	//从分割后的图像里提取障碍物，即提取像素值相同的一块像素点
	for (int i = 0; i < marks.rows; i++)
	{
		for (int j = 0; j < marks.cols; j++)
		{
			int o_pix = marks.at<int>(i, j);
			int index = i * marks.rows + j;							//像素编号下标是从0开始的
			if (o_pix != -1)										//-1为障碍物背景像素值
			{
				Ob[o_pix].pix = o_pix;
				Ob[o_pix].pixID.push_back(index);					//存储像素点集序号
				Ob[o_pix].pix_nums++;
			}

		}
	}

	//计算当前帧障碍物质心
	for (int i = 0; i < PIX_RANGE; i++)
		if (Ob[i].pix != INVALID_PIX)
			Ob[i].barycentre = findBaryCentreOfImage(Ob[i].pixID, Ob[i].pix_nums, marks.cols, marks.rows);

	
	//若为第一帧，则将全部障碍物加入怀疑队列
	if (SuspectObstacleQueue.empty())								
	{
		int count = 0;												//障碍物编号
		for (int i = 0 ; i < PIX_RANGE; i++)
		{
			if (Ob[i].pix != INVALID_PIX)
			{
				count++;
				Ob[i].obstacleID = count;
				SuspectObstacleQueue.push_back(Ob[i]);
			}

		}
	}
	//更新怀疑队列
	else  
	{
		 /************************************************************************
		  * 遍历当前帧障碍物数组，从怀疑队列里寻找质心距离和像素点差距在一个阈值范围内的障碍物
		  * 将其出现次数+1
		  * 还采用了一个移除算法，去掉连续几帧内不出现的障碍物
		  * 这里逻辑可以改一下，外围为怀疑队列，内层为当前障碍物数组，遍历后没有找到的，就让出现次数-1
		  * 若count = 0，则从怀疑队列里移除该障碍物
		  ************************************************************************/
		for (int i = 0; i < SuspectObstacleQueue.size(); i++)
		{
			bool isFind = false;
			for (int j = 0; j < PIX_RANGE; j++)
			{
				if (Ob[j].pix != INVALID_PIX)
				{
					double gap_dis = pow((SuspectObstacleQueue[i].barycentre.x - Ob[j].barycentre.x), 2) +
						pow((SuspectObstacleQueue[i].barycentre.y - Ob[j].barycentre.y), 2);
					double gap_pix = fabs(SuspectObstacleQueue[i].pix_nums - Ob[j].pix_nums);
					if (gap_dis < GAP_DIS && gap_pix < GAP_PIX)
					{
						isFind = true;
						SuspectObstacleQueue[i].appear_counts += 1;
						break;
					}
				}
			}
			// 没找到则出现次数-1，直至为0移除
			if (!isFind)
			{
				SuspectObstacleQueue[i].appear_counts -= 1;
				if (SuspectObstacleQueue[i].appear_counts == 0)
				{
					SuspectObstacleQueue.erase(SuspectObstacleQueue.begin() + i);
					SuspectObstacleQueue.shrink_to_fit();
				}
			}
		}
	}
	free(Ob);
}

//从怀疑队列里提取出出现次数已达3次的障碍物，加入确认队列
void getActualObstacleQueue()
{
	for (int i = 0; i < SuspectObstacleQueue.size(); i++)
	{
		if (SuspectObstacleQueue[i].appear_counts == 3)						//该障碍物再连续3帧中都右出现，则加入确认障碍物队列
		{
			ActualObstacleQueue.push_back(SuspectObstacleQueue[i]);
			//同时从怀疑队列移除该障碍物
			SuspectObstacleQueue.erase(SuspectObstacleQueue.begin() + i);
			SuspectObstacleQueue.shrink_to_fit();
		}
	}

}

Vec3b RandomColor(int value)
{
	value = (value * 100) % 255;  //生成0~255的随机数	
	cv::RNG rng;
	int aa = rng.uniform(0, value);
	int bb = rng.uniform(0, value);
	int cc = rng.uniform(0, value);
	return cv::Vec3b(aa, bb, cc);
}