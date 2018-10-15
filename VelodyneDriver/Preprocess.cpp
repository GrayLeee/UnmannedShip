#include "stdafx.h"
#include "Preprocess.h"

cv::Vec3b Preprocess::RandomColor(int value)
{
	value = (value * 100) % 255;  //生成0~255的随机数	
	cv::RNG rng;
	int aa = rng.uniform(0, value);
	int bb = rng.uniform(0, value);
	int cc = rng.uniform(0, value);
	return cv::Vec3b(aa, bb, cc);
}

//该函数做：滤波，分割，标记障碍物，返回分割后的图像																				
cv::Mat Preprocess::Preprocessing(cv::Mat image)
{
	//膨胀处理
	cv::dilate(image, image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	
	//高斯滤波
	cv::GaussianBlur(image, image,
		cv::Size(5, 5),  // 滤波器尺寸
		1);             // 控制高斯曲线形状的参数
						//灰度化，滤波，Canny边缘检测	

	cv::Mat imageGray;
	cv::cvtColor(image, imageGray, CV_RGB2GRAY);//灰度转换	
												//采用高斯滤波：按距离对邻域像素点加权平均
												//cv::GaussianBlur(imageGray, imageGray,
												//cv::Size(5, 5),  // 滤波器尺寸
												//3);             // 控制高斯曲线形状的参数
												//灰度化，滤波，Canny边缘检测	
	cv::Mat binary;
	cv::threshold(imageGray, binary, 5, 255, 0);//阈值分割原图的灰度图，获得二值图像

	//Canny边缘检测
	cv::Canny(binary, binary, 0, 100);

	//查找轮廓，contours被定义成二维浮点型向量，这里面将来会存储找到的边界的（x,y）坐标，存轮廓点
	vector<vector<cv::Point>> contours;
	//定义的层级。这个在找边界findcontours的时候会自动生成，这里只是给它开辟一个空间。
	vector<cv::Vec4i> hierarchy;
	//就能算出边界的坐标，存在contours里面。
	cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());

	cv::Mat marks(image.size(), CV_32S);   //Opencv分水岭第二个矩阵参数
	marks = cv::Scalar::all(0);
	int index = 0;
	int compCount = 50;
	int count = 0;
	for (; index >= 0; index = hierarchy[index][0], compCount = compCount + 20)
	{
		//对marks进行标记，对不同区域的轮廓进行编号，相当于设置注水点，有多少轮廓，就有多少注水点	
		cv::drawContours(marks, contours, index, cv::Scalar::all(compCount), 1, 8, hierarchy);
		count++;
	}
	cout << "轮廓数：" << count << endl;
	//我们来看一下传入的矩阵marks里是什么东西 
	cv::Mat marksShows;
	cv::convertScaleAbs(marks, marksShows);

	cv::watershed(image, marks);


	//我们再来看一下分水岭算法之后的矩阵marks里是什么东西
	cv::Mat afterWatershed;
	convertScaleAbs(marks, afterWatershed);

	//为了显示分割结果，对可能是障碍物的区域进行颜色填充	
	cv::Mat PerspectiveImage = cv::Mat::zeros(image.size(), CV_8UC3);
	for (int i = 0; i<marks.rows; i++)
	{
		for (int j = 0; j<marks.cols; j++)
		{
			int pix = marks.at<int>(i, j);

			if (marks.at<int>(i, j) == -1)
			{
				PerspectiveImage.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
			}
			else
			{
				PerspectiveImage.at<cv::Vec3b>(i, j) = RandomColor(pix);
			}
		}
	}
	// 返回分割后的图像
	return marks;
}


//更新怀疑队列
void Preprocess::getSuspectObstacleQueue(cv::Mat marks, vector<Msg::ObstacleMoveParameter> &SuspectObstacleQueue)
{
	//初始化一帧内的障碍物数组
	Msg::ObstacleMoveParameter *Ob = new Msg::ObstacleMoveParameter[PIX_RANGE];
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

	int backPix = getBackGroundPix(marks);
	//从分割后的图像里提取障碍物，即提取像素值相同的一块像素点
	for (int i = 0; i < marks.rows; i++)
	{
		for (int j = 0; j < marks.cols; j++)
		{
			int o_pix = marks.at<uchar>(i, j);

			int index = i * marks.cols + j;						//像素编号下标是从0开始的
			if (o_pix != -1 && o_pix != backPix)									//-1为障碍物边界像素值
			{
				Ob[o_pix].pix = o_pix;
				Ob[o_pix].pixID.push_back(index);				//存储像素点集序号
				Ob[o_pix].pix_nums++;
			}

		}
	}

	//计算当前帧每个障碍物的质心
	for (int i = 0; i < PIX_RANGE; i++)
		if (Ob[i].pix != INVALID_PIX)
			Ob[i].barycentre = findBaryCentreOfImage(Ob[i].pixID, Ob[i].pix_nums, marks.cols, marks.rows);

	//若为第一帧，则将全部障碍物加入怀疑队列
	if (SuspectObstacleQueue.empty())
	{
		int count = 0;												//障碍物编号
		for (int i = 0; i < PIX_RANGE; i++)
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

						SuspectObstacleQueue[i].speed = gap_dis / (Ob[j].time - SuspectObstacleQueue[i].time);
						SuspectObstacleQueue[i].angle = atan2f(Ob[j].barycentre.x - SuspectObstacleQueue[i].barycentre.x, Ob[j].barycentre.y - SuspectObstacleQueue[i].barycentre.y);
						
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
	delete[] Ob;
}

//从怀疑队列里提取出出现次数已达3次的障碍物，加入确认队列
void Preprocess::getActualObstacleQueue(vector<Msg::ObstacleMoveParameter> &SuspectObstacleQueue, 
	vector<Msg::ObstacleMoveParameter> &ActualObstacleQueue)
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

//像素点序号转像素坐标
cv::Point2i Preprocess::pixIDToCoordinate(int seq_pix, const int width, const int height) {
	/* Fuction:	Convert the seq_pix to the Coordinate of Pixel.
	* @author:	Jiahao-Chen, LAB b-502.
	* Input:	the seq-number of pixel, the width and the height of Image.
	* Output:	the Coordinate of pixel.
	*/
	cv::Point2i pix_coordinate;

	if (seq_pix < 0 || seq_pix >= width * height)
	{
		cout << "the number of seq_pix is out of range!" << endl;
		exit(0);
	}
	pix_coordinate.y = seq_pix / width;
	pix_coordinate.x = seq_pix % width;
	return pix_coordinate;
}

// 将像素坐标转化成像素点的序号
int Preprocess::coordinateToPixID(cv::Point2i pix_coordinate, const int width, const int height)
{
	/* Fuction:	Convert the Coordinate to the seq_pix of Pixel.
	* @author:	Jiahao-Chen, LAB b-502.
	* Input:	the Coordinate of pixel, the width and the height of Image.
	* Output:	the seq-number of pixel.
	*/
	return (pix_coordinate.y) * width + pix_coordinate.x;
}

// 计算障碍物质心
cv::Point2d Preprocess::findBaryCentreOfImage(vector<int> pixID, int pix_nums, const int width, const int height)
{
	/* Fuction:	Find the Center of Image.
	* @author:	Jiahao-Chen, LAB b-502.
	* Input:	the seq-number buffer of pixel, the number of pixel in area,
	*			the width and the height of Image.
	* Output:	the Center of area.
	*/
	cv::Point2i pix_coordinate;
	cv::Point2d barycentre;
	double sum_x = 0.0, sum_y = 0.0;

	for (int i = 0; i < pix_nums; i++)
	{
		pix_coordinate = pixIDToCoordinate(pixID[i], width, height);
		sum_x += pix_coordinate.x;
		sum_y += pix_coordinate.y;
	}
	barycentre.x = sum_x / pix_nums;
	barycentre.y = sum_y / pix_nums;
	return barycentre;
}

int Preprocess::getBackGroundPix(cv::Mat image)
{
	int intensity[257] = { 0 };
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			int pix = image.at<uchar>(i, j);
			if (image.at<uchar>(i, j) < 0)
				intensity[256] += 1;
			else
				intensity[image.at<uchar>(i, j)] += 1;
		}
	}
	int max = -32768;
	int pix = 0;
	for (int i = 0; i < 257; i++)
	{
		if (max < intensity[i])
		{
			max = intensity[i];
			pix = i;
		}
	}
	return pix;


	return 0;
}
