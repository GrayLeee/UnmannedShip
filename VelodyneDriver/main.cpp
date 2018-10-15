#include "stdafx.h"
#include "VelodyneDriver.h"
#include "PointCloud.h"
#include "Preprocess.h"
#include "PathPlanning.h"

const int UDP_port = 2368;
const string device_ip = "172.16.130.200";
 
void testForProcess();
void unitTestOfPath();
void unitTestOfPath2();
void testAngle();

int main()
{
	testAngle();
	return 0;
}

void testForProcess()
{
	VelodyneDriver vlp_driver(device_ip, UDP_port);

	cv::namedWindow("show", CV_WINDOW_AUTOSIZE);
	// 接收100个包
	for (int i = 0; i < 100; i++)
	{
		Msg::outMsg out;
		Msg::VelodynePackets v_packs;
		vlp_driver.GetPacks(v_packs, i);
		// 转换为Mat图
		PointCloud::ScanToOut(v_packs, out);
		Mat mat = PointCloud::OutToMat(out);

		Mat afterDivi = Preprocess::Preprocessing(mat);

		cv::imshow("show", afterDivi);
 		cv::waitKey(200);


	}
	system("pause");
}

void unitTestOfPath()
{
	cv::Point2i start(1, 1);

	string path = "./images/Obs.jpg";
	Mat myMat = cv::imread(path);	
	myMat = Preprocess::Preprocessing(myMat);

	vector<Msg::ObstacleMoveParameter> Obs;
	Preprocess::getSuspectObstacleQueue(myMat, Obs);
	
	vector<Msg::Force> forces;

	for (int i = 0; i < 1; i++)
		forces.push_back(PathPlanning::SelfToObs(Obs[i], start));
	
	Msg::Force ans = PathPlanning::JointForce(forces);



	printf("value = %f, theta = %f\n", ans.value, ans.theta*180/PI);

}

void unitTestOfPath2()
{
	//构造Mat
	cv::Mat myMat(80, 80, CV_8UC1);

	cv::Point2i start(39, 39);

	uchar *p = myMat.ptr<uchar>(0);
	for (int i = 0; i < 6400; i++)
		p[i] = 0;
	for(int i = 0 ;i <20 ;i++ )
		for (int j = 0; j < 20; j++)
		{
			p[i * 80 + j] = 50;
		}
	for (int i = 0; i <20; i++)
		for (int j = 59; j < 80; j++)
		{
			p[i * 80 + j] = 100;
		}


	vector<Msg::ObstacleMoveParameter> Obs;
	Preprocess::getSuspectObstacleQueue(myMat, Obs);


	vector<Msg::Force> forces;
	for (int i = 0; i < Obs.size(); i++)
	{
		Msg::Force f = PathPlanning::SelfToObs(Obs[i], start);
		forces.push_back(f);
	}

	printf("forces: %d\n", forces.size());


	for (auto force : forces)
	{
		printf("value = %f, theta = %f\n", force.value, force.theta * 180 / PI);
	}
	system("pause");
	//Msg::Force ans = PathPlanning::JointForce(force);

	
}

void testAngle()
{
	cv::Point2i p1(0, 0), p2(2, 2), p3(0, 2), p4(0, -2);
	printf("angle1: %f\n", atan2f(p2.y - p1.y, p2.x - p1.x) * 180 / PI);
	//printf("angle2: %f\n", atan2f(p1.y - p2.y, p1.x - p2.x) * 180 / PI);
	//printf("angle1: %f\n", atan2f(p1.y - p2.y, p1.x - p2.x) * 180 / PI);
	//printf("angle1: %f\n", atan2f(p1.y - p2.y, p1.x - p2.x) * 180 / PI);
}
