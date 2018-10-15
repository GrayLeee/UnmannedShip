#include "stdafx.h"
#include "PointCloud.h"

// 初始化帧数据
void PointCloud::Init_OutMsg(Msg::VelodynePackets& scanMsg, Msg::outMsg& out)
{
	out.head.seq = scanMsg.head.seq;
	out.head.stamp = scanMsg.head.stamp;
	out.field[0].name = "x";
	out.field[0].offset = 0;
	out.field[1].name = "y";
	out.field[1].offset = 4;
	out.field[2].name = "z";
	out.field[2].offset = 8;
	out.field[3].name = "intensity";
	out.field[3].offset = 12;
}

// 转换一帧数据
void PointCloud::ScanToOut(Msg::VelodynePackets& scanMsg, Msg::outMsg& out)
{
	Init_OutMsg(scanMsg, out);
	for (size_t i = 0; i < scanMsg.packs.size(); i++)
	{
		SetPoints(scanMsg.packs[i], out);
	}
}

// 将velodyne包转换为点云数据
void PointCloud::SetPoints(Msg::packets & pkt, Msg::outMsg & out)
{
	float azimuth;
	float azimuth_diff;
	int raw_azimuth_diff;
	float last_azimuth_diff = 0;
	float azimuth_corrected_f;
	int azimuth_corrected;
	float x, y, z;
	float intensity;

	float angle_value[16] =
	{
		-0.2617993877991494f,
		0.017453292519943295f,
		-0.22689280275926285f,
		0.05235987755982989f,
		-0.19198621771937624f,
		0.08726646259971647f,
		-0.15707963267948966f,
		0.12217304763960307f,
		-0.12217304763960307f,
		0.15707963267948966f,
		-0.08726646259971647f,
		0.19198621771937624f,
		-0.05235987755982989f,
		0.22689280275926285f,
		-0.017453292519943295f,
		0.2617993877991494f
	};


	for (int block = 0; block < BLOCKS_PER_PACKET; block++)
	{
		Msg::two_byte tmp1;
		tmp1.byte[0] = pkt.data[block * 100 + 2];
		tmp1.byte[1] = pkt.data[block * 100 + 3];
		azimuth = (float)(tmp1.uint);
		if (block < (BLOCKS_PER_PACKET - 1))
		{
			Msg::two_byte tmp2;
			tmp2.byte[0] = pkt.data[(block + 1) * 100 + 2];
			tmp2.byte[1] = pkt.data[(block + 1) * 100 + 3];
			raw_azimuth_diff = (float)(tmp2.uint) - azimuth;
			azimuth_diff = (float)((36000 + raw_azimuth_diff) % 36000);
			if (raw_azimuth_diff < 0)
			{
				if (last_azimuth_diff > 0)
				{
					azimuth_diff = last_azimuth_diff;
				}
				else
				{
					continue;
				}
			}
			last_azimuth_diff = azimuth_diff;
		}
		else
		{
			azimuth_diff = last_azimuth_diff;
		}
		for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
		{
			for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)
			{
				Msg::two_byte tmp;
				tmp.byte[0] = pkt.data[block * 100 + 4 + k];
				tmp.byte[1] = pkt.data[block * 100 + 4 + k + 1];
				azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
				azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
				float distance = tmp.uint * DISTANCE_RESOLUTION;

				float cos_vert_angle = cos(angle_value[dsr]);
				float sin_vert_angle = sin(angle_value[dsr]);
				float cos_rot_angle = cos((float)(azimuth_corrected) / 18000 * PI);
				float sin_rot_angle = sin((float)(azimuth_corrected) / 18000 * PI);
				float xy_distance = distance * cos_vert_angle;
				y = xy_distance * sin_rot_angle;
				x = xy_distance * cos_rot_angle;
				//if (x < 0) x = -x;
				//if (y < 0) y = -y;

				z = distance * sin_vert_angle;
				float x_coord = -y;
				float y_coord = x;
				float z_coord = z;

				intensity = pkt.data[block * 100 + 4 + k + 2];
				float min_intensity = MIN_INTENSITY;
				float max_intensity = MAX_INTENSITY;
				intensity = (intensity < min_intensity) ? min_intensity : intensity;
				intensity = (intensity > max_intensity) ? max_intensity : intensity;

				if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
				{
					Msg::point v;
					v.intensity = intensity;
					v.x = x_coord;
					v.y = y_coord;
					v.z = z_coord;;
					if (v.x == 0 && v.y == 0 && v.z == 0)
						continue;
					out.points.push_back(v);
				}
			}
		}
	}
	out.width = int(out.points.size());
	out.row_step = out.width * 16;
	if (out.width > 15000)
		out.is_dense = true;
	else
		out.is_dense = false;
}

// 将点云数据映射到预处理的Mat数组里
Msg::MatArray PointCloud::GetMatArray(Msg::outMsg& out)
{
	float unit = 0.05f;
	float length = 40.0f;
	float width = 40.0f;
	int row = (int)(width / unit);
	int column = (int)(length / unit);

	Msg::MatArray myMat;
	myMat.Init(length, width, unit);

	int all = 0;

	for (auto&& point : out.points)
	{
		if (fabs(point.x) < (length / 2) && fabs(point.y) < (width / 2))
		{
			int index_x = row / 2 - (int)(point.y / unit);
			int index_y = column / 2 - (int)(point.x / unit);

			int oldIntensity = myMat.count[index_x][index_y];
			int oldCount = myMat.count[index_x][index_y];
			
			//可能多个激光点投影到一个栅格点，求多个激光点的平均强度
			myMat.matrix[index_x][index_y] = ((int)point.intensity + oldIntensity * oldCount) / (oldCount + 1) ;

			// 累加点
			myMat.count[index_x][index_y] += 1;
		}
	}

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			all += myMat.count[i][j];
		}
	}
	printf("all = %d\n", all);
	return myMat;
}

// 点云数据转Mat数组
Mat PointCloud::OutToMat(Msg::outMsg out)
{
	Msg::MatArray myMat = GetMatArray(out);
	Mat mat(myMat.row, myMat.column, CV_8UC3);
	// 转换
	for (int i = 0; i < myMat.row; i++)
	{
		for (int j = 0; j < myMat.column; j++)
		{
			mat.at<cv::Vec3b>(i, j)[0] = myMat.matrix[i][j];
			mat.at<cv::Vec3b>(i, j)[1] = myMat.matrix[i][j];
			mat.at<cv::Vec3b>(i, j)[2] = myMat.matrix[i][j];
		}
	}
	return mat;
}

