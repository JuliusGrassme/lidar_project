//#pragma once

#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include "segment/bgsegm.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <boost/format.hpp>
#include <mutex>
#include <thread>



//using namespace pcl::tracking;

using namespace cv::bgsegm;
using namespace boost::filesystem;

const std::string SAVEPATH = "E:/Trafikdag/";

using namespace boost::filesystem;

//class custompoint
//{
//public:
//	float xcord;
//	float ycord;
//	float zcord;
//	unsigned char reflectance;
//
//	inline custompoint(float _x, float _y, float _z, float _r)
//	{
//		xcord = _x;
//		ycord = _y;
//		zcord = _z;
//		reflectance = _r;
//	}
//};



class CSVRow
{
public:
	std::string const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readNextRow(std::istream& str)
	{
		std::string         line;
		std::getline(str, line);

		std::stringstream   lineStream(line);
		std::string         cell;

		m_data.clear();
		while (std::getline(lineStream, cell, ','))
		{
			m_data.push_back(cell);
		}
		// This checks for a trailing comma with no data after it.
		if (!lineStream && cell.empty())
		{
			// If there was a trailing comma then add an empty element.
			m_data.push_back("");
		}
	}
private:
	std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
	data.readNextRow(str);
	return str;
}

// this class handelse access to the cloud data 
class clouddata
{

public:

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_frame_pointcloud; // this is just a single frame 

	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloud_data_frames;


	int data_number;
	mutable boost::shared_mutex Cloud_Data_mutex; // the mutex to protect read and write cloud_Data;

	cv::Ptr<cv::BackgroundSubtractor> pGSOC;
	Mat fgMaskGSOC;
	// innit the cloud data
	clouddata()
	{
		cout << "making a clouddata" << endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (float z(-1.0); z <= 1.0; z += 0.05)
		{
			for (float angle(0.0); angle <= 360.0; angle += 5.0)
			{
				pcl::PointXYZRGBA basic_point;
				basic_point.x = 0.25 * cosf(pcl::deg2rad(angle));
				basic_point.y = sinf(pcl::deg2rad(angle));
				basic_point.z = z;
				basic_point.r = 255;
				basic_point.g = 255;
				basic_point.b = 255;
				basic_cloud_ptr->points.push_back(basic_point);
			}
		}


		pGSOC = createBackgroundSubtractorGSOC(LSBP_CAMERA_MOTION_COMPENSATION_LK, 5, 0.003f, 0.01f, 32, 0.01f, 0.0022f, 0.1f, 0.1f, 0.0004f, 0.0008f);

		std::string path1 = ("C:/testdir/img/");

		cv::Mat backframe = imread("C:/testdir/background.png");



		cv::cvtColor(backframe, backframe, CV_RGB2Lab);

		for (int j = 0; j < backframe.rows; ++j)
		{
			for (int i = 0; i < backframe.cols; ++i)
			{
				backframe.at<cv::Vec3b>(j, i)[2] = 128;
			}
		}
		cv::cvtColor(backframe, backframe, CV_Lab2RGB);

		pGSOC->apply(backframe, fgMaskGSOC);
		pGSOC->apply(backframe, fgMaskGSOC);
		pGSOC->apply(backframe, fgMaskGSOC);
		pGSOC->apply(backframe, fgMaskGSOC);
		pGSOC->apply(backframe, fgMaskGSOC);



		current_frame_pointcloud = basic_cloud_ptr;

		data_number = 0;

	}
	

	void remake( float angless )
	{

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (float z(-1.0); z <= angless; z += 0.05)
		{
			for (float angle(0.0); angle <= 360.0; angle += 5.0)
			{
				pcl::PointXYZRGBA basic_point;
				basic_point.x = 1 * cosf(pcl::deg2rad(angle));
				basic_point.y = sinf(pcl::deg2rad(angle));
				basic_point.z = z;
				basic_point.r = 255;
				basic_point.g = 255;
				basic_point.b = 255;
				basic_cloud_ptr->points.push_back(basic_point);
			}
		}




		current_frame_pointcloud = basic_cloud_ptr;

		data_number = 0;

	}


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr read()
	{
		boost::shared_lock<boost::shared_mutex> lock(Cloud_Data_mutex); // it can be shared to read
		//boost::unique_lock<boost::shared_mutex> lock(Cloud_Data_mutex); // it canot be shared to write
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outcloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		*outcloud_ptr = *current_frame_pointcloud;

		return outcloud_ptr;
	}


	void write(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input)
	{
		boost::unique_lock<boost::shared_mutex> lock(Cloud_Data_mutex); // it canot be shared to write
		//cloud_data_frames.push_back(current_frame_pointcloud);
		current_frame_pointcloud = input;
	}


	void add_point(pcl::PointXYZRGBA input)
	{
		boost::unique_lock<boost::shared_mutex> lock(Cloud_Data_mutex); // it canot be shared to write

		current_frame_pointcloud->points.push_back(input);

	}


	void save()
	{
		#pragma region
		std::stringstream ssx;
		ssx << boost::posix_time::microsec_clock::local_time();
		std::string s = ssx.str();
		std::replace(s.begin(), s.end(), '.', ' ');
		std::replace(s.begin(), s.end(), ':', ' ');
		std::stringstream ss;
		ss << SAVEPATH; // whatever its set to
		ss << "video data ";
		ss << data_number;
		ss << " ";
		ss << s;
		ss << ".bin";
		#pragma endregion construct the path where to save the cloud data

		#pragma region
		std::fstream outfile(ss.str(), ios::binary | ios::out | ios::trunc);
		for (long i = 0; i < this->read()->size(); i++)
		{
			//std::vector<CustomPoint>& vr = *cloud_Data_ptr->read(); //Create a reference
			//vr[2].xcord; //Normal access through reference

			outfile.write((char*)&this->read()->at(i).x, sizeof(this->read()->at(i).x)); // xcoord
			outfile.write((char*)&this->read()->at(i).y, sizeof(this->read()->at(i).y)); // ycoord
			outfile.write((char*)&this->read()->at(i).z, sizeof(this->read()->at(i).z)); // zcoord
			outfile.write((char*)&this->read()->at(i).r, sizeof(this->read()->at(i).r)); // redcol
			outfile.write((char*)&this->read()->at(i).g, sizeof(this->read()->at(i).g)); // greencol
			outfile.write((char*)&this->read()->at(i).b, sizeof(this->read()->at(i).b)); // bluecol
			outfile.write((char*)&this->read()->at(i).a, sizeof(this->read()->at(i).r)); // reflectance

			//cout << cloud_Data_ptr->read()->points[i].x << "  " << cloud_Data_ptr->read()->points[i].y << "  " << cloud_Data_ptr->read()->points[i].z;
		}
		outfile.close();
		data_number = data_number + 1;
		#pragma endregion write the data to the file specified in a binary format as this was needed for being able to keep up with

	}


	pcl::PointCloud<pcl::Normal>::Ptr get_clouddata_normals()
	{
		
		// get cloud and convert into a format we can estimate normals on
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 = current_frame_pointcloud;
		pcl::PointCloud<pcl::PointXYZRGBA> incloud = *cloud1;
		pcl::PointCloud<pcl::PointXYZRGB> outcloud;
		pcl::copyPointCloud(incloud, outcloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		*outcloud_ptr = outcloud;
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne1;
		ne1.setInputCloud(outcloud_ptr);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
		ne1.setSearchMethod(tree);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
		ne1.setRadiusSearch(0.1);//-----Calculate surface normals with a search radius of 0.1-----
		ne1.compute(*cloud_normals1);
		return cloud_normals1;
	
	}

	//void load_cloud_csv_testdata1(int amout)
	//{
	//	//"Points_m_XYZ:0",
	//	//"Points_m_XYZ:1",
	//	//"Points_m_XYZ:2",
	//	//"X",
	//	//"Y",
	//	//"Z",
	//	//"intensity",
	//	//"laser_id",
	//	//"azimuth",
	//	//"distance_m",
	//	//"adjustedtime",
	//	//"timestamp",
	//	//"vertical_angle",
	//	//"dual_distance",
	//	//"dual_intensity",
	//	//"dual_return_matching"
	//	//0.00424536969512701,
	//	//4.864833831787109,
	//	//	-2.885083913803101,
	//	//	0.004245369523924373,
	//	//	4.864833888958605,
	//	//	-2.885083917267833,
	//	//	9.0,
	//	//	0.0,
	//	//	5.0,
	//	//	5.656,
	//	//	41011293536.0,
	//	//	1411293536.0,
	//	//	-30.67,
	//	//	0.0,
	//	//	0.0,
	//	//	-1.0
	//	std::string the_path = "E:/Trafik/CSV/";
	//	int number = 19000;
	//	current_frame_pointcloud->points.clear();
	//	for (int i = 0; i < amout; i++)
	//	{
	//		std::stringstream ss;
	//		ss << the_path;
	//		ss << "2019-07-08-13-07-31_Velodyne-HDL-32-Data 19000 - 23000 (Frame ";
	//		ss << std::to_string(number);
	//		ss << ")";
	//		ss << ".csv";
	//		std::cout << "reading:   " << ss.str() << std::endl;
	//		std::ifstream file(ss.str());
	//		CSVRow row;
	//		while (file >> row)
	//		{
	//			atof(row[10].c_str());
	//			pcl::PointXYZRGBA basic_point;
	//			basic_point.x = atof(row[3].c_str());
	//			basic_point.y = atof(row[4].c_str());
	//			basic_point.z = atof(row[5].c_str());
	//			basic_point.r = 255;
	//			basic_point.g = 255;
	//			basic_point.b = 255;
	//			basic_point.a = atoi(row[6].c_str());
	//			current_frame_pointcloud->points.push_back(basic_point);
	//		}
	//		number = number + 1;
	//	}
	//	cloud_data_frames.push_back(current_frame_pointcloud);
	//}
	
	float load_jpg_image_pointcloud( float targert_sec_time )
	{
		float sec_time_image = 0;
		//float targert_sec_time = 1420.5;
		std::string path1 = ("C:/testdir/img/");
		float best_diff_image = 100000000000000;
		float best_time_image = 42;
		std::string best_path_image;

		cout << "gotten target time = " << targert_sec_time << endl;

		for ( directory_entry& entry1 : directory_iterator(path1) )
		{

			std::string path_image = entry1.path().string();
			std::string temppath = path_image;

			//std::cout << temppath << std::endl;
			std::replace(temppath.begin(), temppath.end(), '.', ' ');
			//std::cout << temppath << std::endl;
			std::vector<std::string> result;
			std::stringstream ss(temppath);

			while (ss.good())
			{
				string substr;
				getline(ss, substr, ' ');
				result.push_back(substr);
				//std::cout << substr << std::endl;
			}

			

			sec_time_image = stof(result[6]) + stof(result[5]) * 60 + stof(result[7]) / 1000000;



			//std::cout << "frame " << result[2] << std::endl;
			//std::cout << "hour   "<< result[4] << std::endl;
			//std::cout << "minute " << result[5] << std::endl;
			//std::cout << "second " << sec_time_image << std::endl;

			//std::cout << "ms     " << stoi(result[7])/1000 << std::endl;
			//std::cout << "u sec  " << result[7] << std::endl;

			float diff_image = std::abs(sec_time_image - targert_sec_time);

			//std::cout << "temp path :  " << temppath << "  - " << sec_time_image << " - " << diff_image << std::endl;


			if (diff_image < best_diff_image)
			{
				//std::cout << "Found better image witht diff" << diff_image << std::endl;
				//std::cout << "second " << sec_time_image << std::endl;
				//std::cout << "ms     " << stof(result[7]) / 1000 << std::endl;
				best_diff_image = diff_image;

				best_time_image = sec_time_image;
				best_path_image = path_image;
			}
		}

		cout << "best image time  = " << targert_sec_time << endl;


		cv::Mat imageDistorted = imread(best_path_image);
		//cv::Mat image(imagein);//use of copy constructor

		//Mat Camera_Matrix = Mat(1,9, CV_32FC1);
		//
		//Mat Distortion_Coefficients = Mat(1,4, CV_32FC1);
		//

		//Distortion_Coefficients.at<double>(0) = 654;

		//Camera_Matrix.at<double>(0, 0) = 654;
		//Camera_Matrix.at<double>(1, 0) = 0;
		//Camera_Matrix.at<double>(2, 0) = 0;

		//Camera_Matrix.at<double>(0, 1) = 0;
		//Camera_Matrix.at<double>(1, 1) = 655;
		//Camera_Matrix.at<double>(2, 1) = 0;

		//Camera_Matrix.at<double>(0, 2) = 667;
		//Camera_Matrix.at<double>(1, 2) = 374;
		//Camera_Matrix.at<double>(2, 2) = 1;
		//

		//radial		k1 k2 -0.332326776869161	0.0939194991996419
		//tangiental  p1 p2 0 0



		//read the calibrating parameters.
		
		//const string inputSettingsFile ="C:\testdir\my_intrinsics.yml";
		//
		//FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings


		////Mat Camera_Matrix;
		////Mat Distortion_Coefficients;
		////fs["camera_matrix"] >> Camera_Matrix;
		////fs["distortion_coefficients"] >> Distortion_Coefficients;
		////

		//cout << Camera_Matrix  << endl;
		//cout << Distortion_Coefficients << endl;

		//fs.release();

		//Mat cameraMatrix1 = (Mat_<double>(3, 3, CV_64FC1) << 654.765431176480, 0, 0, 0, 655.095667902479, 0, 667.820313255158, 374.434799759267, 1);
		//Mat distCoeffs1 = (Mat_<double>(4, 1, CV_64FC1) << -0.33, 0.09, 0, 0);








		//Mat cameraMatrixQ = (Mat_<double>(3, 3, CV_64FC1) << 654.765431176480, 0, 1, 0, 655.095667902479, 1, 667.820313255158, 374.434799759267, 1);
		//Mat distCoeffsQ = (Mat_<double>(4, 1, CV_64FC1) << 0, 0, 0, 0);

		//Mat image;
		////Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1);

		////undistort(imageDistorted, imageDistorted, cameraMatrixQ, distCoeffsQ);

		//cv::Size image_size = imageDistorted.size();

		//cout << "d"

		//	cv::Mat map1, map2;
		////cv::initUndistortRectifyMap(cameraMatrixQ, distCoeffsQ, cv::Mat(), cameraMatrixQ, image_size, CV_16SC2, map1, map2);


		//initUndistortRectifyMap(cameraMatrixQ, distCoeffsQ, Mat(), getOptimalNewCameraMatrix(cameraMatrixQ, distCoeffsQ, image_size, 1, image_size, 0), image_size, CV_16SC2, map1, map2);



		//cout << imageDistorted.type() << endl;


		////cout << map2 << endl;


		//cv::remap(imageDistorted, image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

		//cv::imshow("Original", imageDistorted);
		//cv::imshow("Undistorted", image);

		//cout << imageDistorted.type() << endl;
		//cout << imageDistorted.type() << endl;

		//waitKey();




















		Mat image;

		//undistort(imageDistorted, image, cameraMatrix1, distCoeffs1);
		
		image = imageDistorted;






















#pragma region

		cv::cvtColor(image, image, CV_RGB2Lab);

		for (int j = 0; j < image.rows; ++j)
		{
			for (int i = 0; i < image.cols; ++i)
			{
				image.at<cv::Vec3b>(j, i)[2] = 128;
			}
		}
		cv::cvtColor(image, image, CV_Lab2RGB);

		pGSOC->apply(image, fgMaskGSOC);

		int morph_size = 2;
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(1 * morph_size + 1, 1 * morph_size + 1), Point(morph_size, morph_size));

		int morph_size2 = 3;
		Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(1 * morph_size2 + 1, 1 * morph_size2 + 1), Point(morph_size2, morph_size2));

		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_DILATE, element);

		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);
		morphologyEx(fgMaskGSOC, fgMaskGSOC, MORPH_ERODE, element);

		Mat canny_output;
		Mat fgMaskGSOC_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		cvtColor(fgMaskGSOC, fgMaskGSOC_output, CV_GRAY2BGR);

		Canny(fgMaskGSOC_output, canny_output, 50, 100, 3);

		findContours(canny_output, contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
		for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(255, 255, 255);
			drawContours(drawing, contours, i, color, 1, 8, hierarchy, INT_MAX, Point(-1, -1));
		}

		fillPoly(drawing, contours, cv::Scalar::all(255), 8);
		morphologyEx(drawing, drawing, MORPH_DILATE, element);
		morphologyEx(drawing, drawing, MORPH_DILATE, element);

		cv::Mat marks = imread("C:/testdir/markings.png");

		cv::Mat outframe = Mat::zeros(image.size(), CV_8UC3);

		for (int j = 0; j < image.rows; ++j)
		{
			for (int i = 0; i < image.cols; ++i)
			{
				if (fgMaskGSOC.at<cv::Vec3b>(j, i / 3)[0] == 255 || drawing.at<cv::Vec3b>(j, i)[0] == 255)
				{


					if (marks.at<cv::Vec3b>(j, i)[0] == 0)
					{
						outframe.at<cv::Vec3b>(j, i)[0] = 255;
						outframe.at<cv::Vec3b>(j, i)[1] = 255;
						outframe.at<cv::Vec3b>(j, i)[2] = 255;

						//image.at<cv::Vec3b>(j, i)[0] = 0;
						//image.at<cv::Vec3b>(j, i)[1] = 0;
						//image.at<cv::Vec3b>(j, i)[2] = 0;


					}
				}
				//if (marks.at<cv::Vec3b>(j, i)[0] == 255)
				//{
				//	image.at<cv::Vec3b>(j, i)[0] = 255;
				//	image.at<cv::Vec3b>(j, i)[1] = 0;
				//	image.at<cv::Vec3b>(j, i)[2] = 0;
				//}
			}
		}

		//imshow("Image", image);
		//imshow("Mask", outframe);
		//waitKey(10);


#pragma endregion




		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

		//cout << "the image d type" << imageDistorted.type() << endl;
		//cout << "the image u type" << image.type() << endl;

		//int image_rows = 0;
		//int image_cols = 0;
		//for (int r = 0; r < image.rows; ++r) 
		//{
		//	cv::Point3_<uint8_t>* ptr = image.ptr<cv::Point3_<uint8_t>>(r, 0);
		//	const cv::Point3_<uint8_t>* ptr_end = ptr + image.cols;
		//	for (; ptr != ptr_end; ++ptr) 
		//	{
		//		ptr->x = 255;
		//		ptr->y = 255;
		//		ptr->z = 255;
		//		pcl::PointXYZRGBA basic_point;
		//		basic_point.x = image_rows;
		//		basic_point.y = image_cols;
		//		image_cols = image_cols + 1;
		//		basic_point.z = 0;
		//		basic_point.r = ptr->x;
		//		basic_point.g = ptr->y;
		//		basic_point.b = ptr->z;
		//		basic_point.a = 255;
		//		basic_cloud_ptr->points.push_back(basic_point);
		//		
		//	}
		//	image_rows = image_rows + 1;
		//}

		//cv::Size imgSize(640, 480);
		//cv::Mat image = cv::Mat(imgSize, CV_8UC3, cv::Scalar(1, 1, 1));

		for (int row = 0; row < image.rows; ++row)
		{
			const uchar* ptr = image.ptr(row);
			const uchar* mask_ptr = outframe.ptr(row);

			for (int col = 0; col < image.cols; col++)
			{
				const uchar* uc_pixel = ptr;

				const uchar* mask_uc_pixel = mask_ptr;

				int a = uc_pixel[0];
				int b = uc_pixel[1];
				int c = uc_pixel[2];

				int z = mask_uc_pixel[0];
				int zz = mask_uc_pixel[1];
				int zzz = mask_uc_pixel[2];

				//cout << z << " " << zz << " " << zzz << endl;

				if ( z == 255 && zz == 255 && zzz == 255 )
				{
					pcl::PointXYZRGBA basic_point;
					float pix_size_var_x = 0.0190;
					float pix_size_var_y = 0.0220;
					basic_point.x = -col * pix_size_var_x + (image.cols * pix_size_var_x) / 2;
					basic_point.y = -row * pix_size_var_y;// +(image.rows * pix_size_var_y) / 2; 
					basic_point.z = 0;

					basic_point.x = basic_point.x + 0;
					basic_point.y = basic_point.y + 6;

					basic_point.r = c*0.5;
					basic_point.g = b*0.5;
					basic_point.b = a*0.5;

					basic_point.a = 255;

					basic_cloud_ptr->points.push_back(basic_point);
				}
				else
				{
					pcl::PointXYZRGBA basic_point;
					float pix_size_var_x = 0.0190;
					float pix_size_var_y = 0.0220;
					basic_point.x = -col * pix_size_var_x + (image.cols * pix_size_var_x) / 2;
					basic_point.y = -row * pix_size_var_y;// +(image.rows * pix_size_var_y) / 2; 
					basic_point.z = 0;

					basic_point.x = basic_point.x + 0;
					basic_point.y = basic_point.y + 6;

					basic_point.r = c;
					basic_point.g = b;
					basic_point.b = a;

					basic_point.a = 255;

					basic_cloud_ptr->points.push_back(basic_point);
				}





				ptr += 3;
				mask_ptr += 3;



			}
		}





	#pragma region

		



		//std::vector<cv::Point3f> objectPoints;

		//for (int i = 0; i < basic_cloud_ptr->points.size(); i++)
		//{
		//	// the points should proberly be between -0.5 and 0.5 which they are not 

		//	cv::Point3f dsa = cv::Point3f(basic_cloud_ptr->points[i].x , basic_cloud_ptr->points[i].y , basic_cloud_ptr->points[i].z );

		//	objectPoints.push_back(dsa);
		//}


		//std::vector<cv::Point2f> projectedPoints;


		//Mat cameraMatrix1 = (Mat_<double>(3, 3, CV_64FC1) << 654.765431176480, 0, 0, 0, 655.095667902479, 0, 667.820313255158, 374.434799759267, 1);
		//Mat distCoeffs1 = (Mat_<double>(5, 1, CV_64FC1) << -0.20, 0.20, 0.0, 0.0, 0.0);

		//projectPoints(objectPoints, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), cameraMatrix1, distCoeffs1, projectedPoints);



		//for (int i = 0; i < projectedPoints.size(); i++)
		//{
		//		


		//		pcl::PointXYZRGBA basic_point;

		//		basic_point.x = projectedPoints[i].x;
		//		basic_point.y = projectedPoints[i].y;
		//		basic_point.z = 0;

		//		basic_point.r = projectedPoints[i];
		//		basic_point.g = b;
		//		basic_point.b = a;
		//		basic_point.a = 255;
		//		basic_cloud_ptr->points.push_back(basic_point);


		//}

	#pragma endregion











		current_frame_pointcloud = basic_cloud_ptr;


		cout << "return best time image" << best_time_image << endl;
		return best_time_image;

	}


	float load_csv_cloud_pointcloud(float targert_sec_time, bool do_projection, bool do_add_image)
	{
		//float sec_time_cloud = 0;
		#pragma region
		//float targert_sec_time = 1420.5;

		cout << "cloud data load target point " << targert_sec_time << endl;

		std::string path2 = ("C:/testdir/csv/");
		float best_diff_cloud = 100000000000000;
		float best_time_cloud = 42;
		std::string best_path_cloud;
		for ( directory_entry& entry2 : directory_iterator(path2) )
		{
			//std::cout << "tjeck dir cloud" << std::endl;
			std::string path_cloud = entry2.path().string();

			std::string temppath = path_cloud;
			std::replace(temppath.begin(), temppath.end(), '_', '-');
			std::replace(temppath.begin(), temppath.end(), '(', '-');
			std::replace(temppath.begin(), temppath.end(), ')', '-');
			std::replace(temppath.begin(), temppath.end(), ' ', '-');
			std::replace(temppath.begin(), temppath.end(), '.', '-');
			//std::cout << temppath << std::endl;
			std::vector<std::string> result;
			std::stringstream ss(temppath);

			while (ss.good())
			{
				string substr;
				getline(ss, substr, '-');
				result.push_back(substr);
				//std::cout << substr << std::endl;
			}

			std::ifstream file(path_cloud);
			CSVRow row;
			double avg_time = 0;
			int numss = 0;

			//int runs = 0;
			//while (file >> row)
			//{	
			//	if (runs == 0)
			//	{
			//		
			//	}
			//	else
			//	{
			//		//std::cout << "time" << stoi(row[11]) << std::endl;
			//		if ( runs == 1 )
			//		{
			//			avg_time = avg_time + stoi(row[11]);
			//			numss = numss + 1;
			//		}
			//	}
			//	runs = runs + 1;
			//}

			file >> row;
			file >> row;
			avg_time = +stoi(row[11]);
			numss = 1;

			//std::cout << "frame  " << result[16] << std::endl;
			//std::cout << "hour   " << result[3] << std::endl;
			//std::cout << "minute " << ( (avg_time / numss) / 1000000 ) / 60 << std::endl;

			float sec_time_cloud = (avg_time / numss) / 1000000;

			//std::cout << "second " << sec_time_cloud << std::endl;

			//std::cout << "ms     " << (avg_time / numss) / 1000 << std::endl;
			//std::cout << "u sec  " << (avg_time / numss) << std::endl;

			float diff_cloud = std::abs(sec_time_cloud - targert_sec_time);
			//std::cout << temppath << " time" << sec_time_cloud << "  diff " << diff_cloud << std::endl;


			if (diff_cloud < best_diff_cloud)
			{
				//std::cout << "new best_diff_cloud " << diff_cloud << std::endl;
				best_diff_cloud = diff_cloud;
				best_time_cloud = sec_time_cloud;
				best_path_cloud = path_cloud;
			}



		}
		#pragma endregion

		std::cout << "best_time_cloud " << best_time_cloud << std::endl;

		#pragma region

		std::ifstream file(best_path_cloud);
		CSVRow row;
		file >> row; // read in the top header which only contains descriptors
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

		//Eigen::Quaterniond quat(0.16, 0, 0, 1);

		//These numbers come from this property:

		float a = 0.01745329251994329576923690768489 * -2;

		float nx = 0; // around x axis 
		float ny = 1;
		float nz = 0;
		
		//[w, x, y, z] = [
		float w = cos(a / 2);
		float x = sin(a / 2) * nx;
		float y = sin(a / 2) * ny;
		float z = sin(a / 2) * nz;
		//]
		//Where a is the angle of rotation and{ nx,ny,nz } is the axis of rotation. (From section 2e of Quaternion Spells)
		Eigen::Quaterniond quat(w, x, y, z);


		while(file >> row) // read numbers
		{	


			Eigen::Vector3d pnt( stof(row[3].c_str()), stof(row[4].c_str()), stof(row[5].c_str()) );

			
			Eigen::Vector3d pntRot = pnt ; // rotate pnt by the quaternion

			//cout << "beff : " << pntRot << endl;

			pntRot = quat * pnt;

			//cout << "afft : " << pntRot << endl;

			// im matrix form
			//Matrix3Xd in_pts;
			//Matrix3Xd out_pts;
			//Affine3d T = quats * Translation3d(-Trans);
			//out_pts = T * in_popents;

			pcl::PointXYZRGBA basic_point;
			basic_point.x = -pntRot.coeff(1);
			basic_point.y = pntRot.coeff(2);
			basic_point.z = -pntRot.coeff(0);  

			basic_point.r = 255;
			basic_point.g = 255;
			basic_point.b = 255;
			//basic_point.a = atoi(row[6].c_str());
			basic_point.a = 255;

			if (basic_point.z > 0)
			{
				basic_cloud_ptr->points.push_back(basic_point);
			}

			
			
		}


		#pragma region

		std::vector<cv::Point2f> imagePoints;

		imagePoints.push_back(cv::Point2f(282, 274));

		std::vector<cv::Point3f> objectPoints;

		double large_x = 0;
		double large_y = 0;
		double large_z = 0;
		double small_x = 0;
		double small_y = 0;
		double small_z = 0;
		for (int i = 0; i < basic_cloud_ptr->points.size(); i++)
		{

			if (basic_cloud_ptr->points[i].x > large_x )
			{
				large_x = basic_cloud_ptr->points[i].x;
			}

			if (basic_cloud_ptr->points[i].y > large_y)
			{
				large_y = basic_cloud_ptr->points[i].y;
			}

			if (basic_cloud_ptr->points[i].z > large_z)
			{
				large_z = basic_cloud_ptr->points[i].z;
			}

			if (basic_cloud_ptr->points[i].x < small_x)
			{
				small_x = basic_cloud_ptr->points[i].x;
			}

			if (basic_cloud_ptr->points[i].y < small_y)
			{
				small_y = basic_cloud_ptr->points[i].y;
			}

			if (basic_cloud_ptr->points[i].z < small_z)
			{
				small_z = basic_cloud_ptr->points[i].z;
			}

		}

		double arr[7];
		 
		arr[1] = large_x;
		arr[2] = large_y;
		arr[3] = large_z;

		arr[4] = small_x;
		arr[5] = small_y;
		arr[6] = small_z;
		
		for (int i = 1; i < 7; ++i)
		{
			//cout << arr[0] << "  dsa  " << arr[i] << endl;
			if (arr[0] < arr[i])
				arr[0] = arr[i];
		}

		//cout << arr[0] << "  laregs  " << endl;

		arr[0] = 1000;


		


		//// Create the known projection matrix
		//cv::Mat P(3, 4, cv::DataType<double>::type);
		//P.at<double>(0, 0) = -2.8058e-01;
		//P.at<double>(1, 0) = -6.8326e-02;
		//P.at<double>(2, 0) = 5.1458e-07;

		//P.at<double>(0, 1) = 2.0045e-02;
		//P.at<double>(1, 1) = -3.1718e-01;
		//P.at<double>(2, 1) = 4.5840e-06;

		//P.at<double>(0, 2) = 1.8102e-01;
		//P.at<double>(1, 2) = -7.2974e-02;
		//P.at<double>(2, 2) = 2.6699e-06;

		//P.at<double>(0, 3) = 6.6062e-01;
		//P.at<double>(1, 3) = 5.8402e-01;
		//P.at<double>(2, 3) = 1.5590e-03;

		//// Decompose the projection matrix into:
		//cv::Mat K(3, 3, cv::DataType<double>::type); // intrinsic parameter matrix
		//cv::Mat rvec(3, 3, cv::DataType<double>::type); // rotation matrix

		//cv::Mat Thomogeneous(4, 1, cv::DataType<double>::type); // translation vector

		//cv::decomposeProjectionMatrix(P, K, rvec, Thomogeneous);

		//cv::Mat T(3, 1, cv::DataType<double>::type); // translation vector
		////cv::Mat T;
		//cv::convertPointsHomogeneous(Thomogeneous, T);

		//std::cout << "K: " << K << std::endl;
		//std::cout << "rvec: " << rvec << std::endl;
		//std::cout << "T: " << T << std::endl;

		//// Create zero distortion
		//cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
		//distCoeffs.at<double>(0) = 0;
		//distCoeffs.at<double>(1) = 0;
		//distCoeffs.at<double>(2) = 0;
		//distCoeffs.at<double>(3) = 0;
		
		for (int i = 0; i < basic_cloud_ptr->points.size(); i++)
		{
			// the points should proberly be between -0.5 and 0.5 which they are not 

			cv::Point3f dsa = cv::Point3f(basic_cloud_ptr->points[i].x / arr[0], basic_cloud_ptr->points[i].y / arr[0], basic_cloud_ptr->points[i].z / arr[0]);

			objectPoints.push_back(dsa);
		}


		std::vector<cv::Point2f> projectedPoints;

		//cv::Mat rvecR(3, 1, cv::DataType<double>::type);//rodrigues rotation matrix
		//cv::Rodrigues(rvec, rvecR);


		// thiese are the intrinsic parameters for 720x1280

		//Mat cameraMatrix1 = (Mat_<double>(3, 3, CV_64FC1) << 654.765431176480, 0, 0, 0, 655.095667902479, 0, 667.820313255158, 374.434799759267, 1);
		//Mat distCoeffs1 = (Mat_<double>(4, 1, CV_64FC1) <<-0.33, 0.09, 0, 0);

		Mat cameraMatrix1 = (Mat_<double>(3, 3, CV_64FC1) << 654.765431176480, 0, 0, 0, 655.095667902479, 0, 667.820313255158, 374.434799759267, 1);
		Mat distCoeffs1 = (Mat_<double>(5, 1, CV_64FC1) <<-0.20, 0.20, 0.0, 0.0, 0.0);

		projectPoints(objectPoints, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), cameraMatrix1, distCoeffs1, projectedPoints);

		//cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);

		large_x = 0;
		large_y = 0;
		large_z = 0;

		small_x = 0;
		small_y = 0;
		small_z = 0;

		for (int i = 0; i < projectedPoints.size(); i++)
		{

			if (projectedPoints[i].x > large_x)
			{
				large_x = projectedPoints[i].x;
			}

			if (projectedPoints[i].y > large_y)
			{
				large_y = projectedPoints[i].y;
			}

			//if (basic_cloud_ptr->points[i].z > large_z)
			//{
			//	large_z = basic_cloud_ptr->points[i].z;
			//}

			if (projectedPoints[i].x < small_x)
			{
				small_x = projectedPoints[i].x;
			}

			if (projectedPoints[i].y < small_y)
			{
				small_y = projectedPoints[i].y;
			}

			//if (basic_cloud_ptr->points[i].z < small_z)
			//{
			//	small_z = basic_cloud_ptr->points[i].z;
			//}

		}

		arr[7];

		arr[1] = large_x;
		arr[2] = large_y;
		arr[3] = large_z;

		arr[4] = small_x;
		arr[5] = small_y;
		arr[6] = small_z;

		for (int i = 1; i < 7; ++i)
		{
			//cout << arr[0] << "  dsa  " << arr[i] << endl;
			if (arr[0] < arr[i])
				arr[0] = arr[i];
		}

		//cout << arr[0] << "  laregs  " << endl;












		#pragma endregion

		for (unsigned int i = 0; i < projectedPoints.size(); ++i)
		{
			//std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;



			if (do_projection)
			{

				basic_cloud_ptr->points[i].x = (projectedPoints[i].x) / 64;
				basic_cloud_ptr->points[i].y = (projectedPoints[i].y) / 64;
				basic_cloud_ptr->points[i].z = 0;
			}
			else
			{

			}

		}




		//for (unsigned int i = 0; i < projectedPoints.size(); ++i)
		//{
		//	std::cout 
		//		<< "x: " << basic_cloud_ptr->points[i].x
		//		<< "y: " << basic_cloud_ptr->points[i].y
		//		<< "z: " << basic_cloud_ptr->points[i].z
		//		<< std::endl;
		//}




		current_frame_pointcloud = basic_cloud_ptr;


		if (do_add_image)
		{

			//float sec_time_image = 0;
			//std::string path2 = ("C:/testdir/img/");
			//float best_diff_image = 100000000000000;
			//float best_time_image = 42;
			//std::string best_path_image;
			//std::cout << "before entry " << std::endl;
			//for (directory_entry& entry2 : directory_iterator(path2))
			//{
			//	std::string path_image = entry2.path().string();
			//	std::string temppath = path_image;
			//	std::replace(temppath.begin(), temppath.end(), '.', ' ');
			//	std::vector<std::string> result;
			//	std::stringstream ss(temppath);
			//	while (ss.good())
			//	{
			//		string substr;
			//		getline(ss, substr, ' ');
			//		result.push_back(substr);
			//	}
			//	sec_time_image = stof(result[6]) + stof(result[5]) * 60 + stof(result[7]) / 1000000;
			//	float diff_image = std::abs(sec_time_image - targert_sec_time);
			//	if (diff_image < best_diff_image)
			//	{
			//		std::cout << "Found better image witht diff" << diff_image << std::endl;
			//		best_diff_image = diff_image;
			//		best_time_image = sec_time_image;
			//		best_path_image = path_image;
			//	}
			//}

			//cv::Mat image = imread(best_path_image);
			//std::cout << "after image" << std::endl;

			//for (int row = 0; row < image.rows; ++row)
			//{
			//	const uchar* ptr = image.ptr(row);
			//	for (int col = 0; col < image.cols; col++)
			//	{
			//		const uchar * uc_pixel = ptr;
			//		int a = uc_pixel[0];
			//		int b = uc_pixel[1];
			//		int c = uc_pixel[2];

			//		pcl::PointXYZRGBA basic_point;
			//		basic_point.x = -col * 0.02 + (image.cols * 0.02) / 2;
			//		basic_point.y = -row * 0.02 + (image.rows * 0.02) / 2;
			//		basic_point.z = 0;

			//		basic_point.r = c;
			//		basic_point.g = b;
			//		basic_point.b = a;
			//		basic_point.a = 255;
			//		//basic_cloud_ptr->points.push_back(basic_point);

			//		add_point(basic_point);

			//		ptr += 3;
			//	}
			//}

			//std::cout << "after push" << std::endl;

		}


		#pragma endregion


		return best_time_cloud;

	}


	void load_csv_cloud_pointcloud_single(std::string the_path)
	{

		std::ifstream file(the_path);
		CSVRow row;
		file >> row; // read in the top header which only contains descriptors
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

		float a = 0.01745329251994329576923690768489 * -2;

		float nx = 0; // around x axis 
		float ny = 1;
		float nz = 0;

		float w = cos(a / 2);
		float x = sin(a / 2) * nx;
		float y = sin(a / 2) * ny;
		float z = sin(a / 2) * nz;

		//Where a is the angle of rotation and{ nx,ny,nz } is the axis of rotation. (From section 2e of Quaternion Spells)
		Eigen::Quaterniond quat(w, x, y, z);

		while (file >> row) // read numbers
		{
			Eigen::Vector3d pnt(stof(row[3].c_str()), stof(row[4].c_str()), stof(row[5].c_str()));
			Eigen::Vector3d pntRot = pnt; // rotate pnt by the quaternion
			pntRot = quat * pnt;
			pcl::PointXYZRGBA basic_point;
			basic_point.x = -pntRot.coeff(1);
			basic_point.y = pntRot.coeff(2);
			basic_point.z = -pntRot.coeff(0);
			basic_point.r = 255;
			basic_point.g = 255;
			basic_point.b = 255;
			basic_point.a = 255;
			if (basic_point.z > 0)
			{
				basic_cloud_ptr->points.push_back(basic_point);
			}
		}


		current_frame_pointcloud = basic_cloud_ptr;
		cout << "loaded single pointcloud with points " << basic_cloud_ptr->points.size() << endl;

	}


	void subtract_background(std::string the_path)
	{

		std::ifstream file(the_path);
		CSVRow row;
		file >> row; // read in the top header which only contains descriptors
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

		float a = 0.01745329251994329576923690768489 * -2;

		float nx = 0; // around x axis 
		float ny = 1;
		float nz = 0;

		float w = cos(a / 2);
		float x = sin(a / 2) * nx;
		float y = sin(a / 2) * ny;
		float z = sin(a / 2) * nz;

		//Where a is the angle of rotation and{ nx,ny,nz } is the axis of rotation. (From section 2e of Quaternion Spells)
		Eigen::Quaterniond quat(w, x, y, z);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

		while (file >> row) // read numbers
		{
			Eigen::Vector3d pnt(stof(row[3].c_str()), stof(row[4].c_str()), stof(row[5].c_str()));
			Eigen::Vector3d pntRot = pnt; // rotate pnt by the quaternion
			pntRot = quat * pnt;
			pcl::PointXYZRGBA basic_point;
			basic_point.x = -pntRot.coeff(1);
			basic_point.y = pntRot.coeff(2);
			basic_point.z = -pntRot.coeff(0);
			basic_point.r = 255;
			basic_point.g = 255;
			basic_point.b = 255;
			basic_point.a = 255;
			if (basic_point.z > 0)
			{
				basic_cloud_ptr->points.push_back(basic_point);
				basic_point.r = 0;
				basic_point.g = 0;
				//out_cloud_ptr->points.push_back(basic_point);
			}
		}


		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud(basic_cloud_ptr);
		kdtree.setEpsilon(0);

		// for each of the points in the current frame
		for (int i = 0; i < current_frame_pointcloud->points.size(); i++)
		{
			if (i % 100000 == 0)
			{
				cout << i << endl;
			}

			// define it as search point 
			pcl::PointXYZRGBA searchPoint;
			searchPoint.x = current_frame_pointcloud->points[i].x;
			searchPoint.y = current_frame_pointcloud->points[i].y;
			searchPoint.z = current_frame_pointcloud->points[i].z;

			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			
			float x1 = current_frame_pointcloud->points[i].x;
			float y1 = current_frame_pointcloud->points[i].y;
			float z1 = current_frame_pointcloud->points[i].z;

			double min_dist = 10000000;

			for (int z = 0; z < basic_cloud_ptr->size(); z++)
			{
				float x2 = basic_cloud_ptr->points[z].x;
				float y2 = basic_cloud_ptr->points[z].y;
				float z2 = basic_cloud_ptr->points[z].z;

				double n_my_dist = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));

				if (n_my_dist < min_dist)
				{
					min_dist = n_my_dist;
				}
			}
			if (min_dist > 0.30)
			{
				pcl::PointXYZRGBA basic_point;
				basic_point.x = current_frame_pointcloud->points[i].x;
				basic_point.y = current_frame_pointcloud->points[i].y;
				basic_point.z = current_frame_pointcloud->points[i].z;
				basic_point.r = 255;
				basic_point.g = 0;
				basic_point.b = 0;
				out_cloud_ptr->points.push_back(basic_point);
			}


			//and search for the nearest point in the background
			//if it is a large enough distance add to new point to the updated cloud
			//if it is to small don't

			// if there is a point in that search radius it means that the point is likely part of the background so do nothing
			//
			//int kdnum = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			////cout << "kdnum " << kdnum << endl;
			//if (kdnum > 0)
			//{
			//	double my_dist = 0;

			//	for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
			//	{

			//		//cout << "epsilon = " << kdtree.getEpsilon() << endl;

			//		//cout << "           x " << current_frame_pointcloud->points[pointIdxNKNSearch[j]].x << endl;
			//		//cout << "           y " << current_frame_pointcloud->points[pointIdxNKNSearch[j]].y << endl;
			//		//cout << "           z " << current_frame_pointcloud->points[pointIdxNKNSearch[j]].z << endl;
			//		
			//		//cout << "           x " << basic_cloud_ptr->points[pointIdxNKNSearch[j]].x << endl;
			//		//cout << "           y " << basic_cloud_ptr->points[pointIdxNKNSearch[j]].y << endl;
			//		//cout << "           z " << basic_cloud_ptr->points[pointIdxNKNSearch[j]].z << endl;

			//		float x1 = current_frame_pointcloud->points[ i ].x;
			//		float y1 = current_frame_pointcloud->points[ i ].y;
			//		float z1 = current_frame_pointcloud->points[ i ].z;

			//		float x2 = basic_cloud_ptr->points[ pointIdxNKNSearch[j] ].x;
			//		float y2 = basic_cloud_ptr->points[ pointIdxNKNSearch[j] ].y;
			//		float z2 = basic_cloud_ptr->points[ pointIdxNKNSearch[j] ].z;

			//		my_dist = sqrt( (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1) );

			//		if (my_dist > 1)
			//		{
			//			//cout << "current_dist " << my_dist << endl;
			//			pcl::PointXYZRGBA basic_point;
			//			basic_point.x = current_frame_pointcloud->points[i].x;
			//			basic_point.y = current_frame_pointcloud->points[i].y;
			//			basic_point.z = current_frame_pointcloud->points[i].z;
			//			basic_point.r = 255;
			//			basic_point.g = 0;
			//			basic_point.b = 0;
			//			basic_point.a = 255;
			//			//out_cloud_ptr->points.push_back(basic_point);
			//		}

			//		//cout << "current_dist " << my_dist << endl;
			//		pcl::PointXYZRGBA basic_point;
			//		basic_point.x = current_frame_pointcloud->points[i].x;
			//		basic_point.y = current_frame_pointcloud->points[i].y;
			//		basic_point.z = current_frame_pointcloud->points[i].z;
			//		basic_point.r = 255;
			//		basic_point.g = 255;
			//		basic_point.b = 255;
			//		basic_point.a = 255;
			//		//out_cloud_ptr->points.push_back(basic_point);

			//		
			//	}










			//}
		}

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(out_cloud_ptr);
		sor.setMeanK(5);
		sor.setStddevMulThresh(0.1/2);
		sor.filter(*cloud_filtered);




		current_frame_pointcloud = cloud_filtered;
	}


	void save_obj()
	{




		//const std::string SAVEPATH = "C:/testdir/obj/";
		//const string logpath = SAVEPATH + "obj_file.obj";
		//std::ofstream log_file(logpath, ios::out | ios::binary | std::ios_base::trunc);

		//// for writing the points into an obj file 

		//log_file << std::fixed << std::setprecision(6);

		//log_file << '\n';

		////log_file
		////	<< "v "
		////	<< 0
		////	<< " "
		////	<< 0
		////	<< " "
		////	<< 0
		////	<< '\n';

		////log_file
		////	<< "v "
		////	<< 1
		////	<< " "
		////	<< 1
		////	<< " "
		////	<< 1
		////	<< '\n';

		//for (int i = 0; i < current_frame_pointcloud->points.size(); i++)
		//{
		//	log_file
		//		<< "v "
		//		<< current_frame_pointcloud->points[i].x + 0.2
		//		<< " "
		//		<< current_frame_pointcloud->points[i].y
		//		<< " "
		//		<< current_frame_pointcloud->points[i].z
		//		<< '\n';

		//	log_file
		//		<< "v "
		//		<< current_frame_pointcloud->points[i].x
		//		<< " "
		//		<< current_frame_pointcloud->points[i].y + 0.2
		//		<< " "
		//		<< current_frame_pointcloud->points[i].z
		//		<< '\n';

		//	log_file
		//		<< "v "
		//		<< current_frame_pointcloud->points[i].x
		//		<< " "
		//		<< current_frame_pointcloud->points[i].y
		//		<< " "
		//		<< current_frame_pointcloud->points[i].z + 0.2
		//		<< '\n';
		//}

		//log_file << '\n';

		//int num_spoints = current_frame_pointcloud->points.size() * 3;

		//for (int i = 1; i < num_spoints; i = i + 3)
		//{

		//	int a_int = i;
		//	int b_int = i + 1;
		//	int c_int = i + 2;

		//	log_file << "f " << a_int << " " << b_int << " " << c_int << '\n';

		//}

		//log_file << '\n';




	}


	void load_the_fused_point_clouds
	(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image_cloud,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projected_cloud,
		bool do_interpolation
	)
	{
		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud(projected_cloud);

		// for all of the pixels in the image 
		for (int i = 0; i < image_cloud->points.size(); i++)
		{


			if (image_cloud->points[i].r > 0 || image_cloud->points[i].g > 0 || image_cloud->points[i].b > 0)
			{

				if (i % 100000 == 0)
				{
					cout << i << endl;
				}

				pcl::PointXYZRGBA interpolatedPoint;

				// extract the point position of the pixel
				double x1 = image_cloud->points[i].x;
				double y1 = image_cloud->points[i].y;
				double z1 = image_cloud->points[i].z;

				// set the pixel searchpoint 
				pcl::PointXYZRGBA searchPoint;
				searchPoint.x = x1;
				searchPoint.y = y1;
				searchPoint.z = z1;

				// set k and make lists for distances and indexes
				int K = 1;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);

				double average_x = 0;
				double average_y = 0;
				double average_z = 0;

				double average_r = 0;
				double average_g = 0;
				double average_b = 0;

				float current_dist = 100000;


				if (do_interpolation)
				{

					if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
						{
							

							//if
							//(
							//	std::abs(original_cloud->points[pointIdxNKNSearch[j]].x - searchPoint.x) < 5.0f ||
							//	std::abs(original_cloud->points[pointIdxNKNSearch[j]].y - searchPoint.y) < 5.0f ||
							//	std::abs(original_cloud->points[pointIdxNKNSearch[j]].z - searchPoint.z) < 5.0f
							//)
							//{
								average_x = average_x + original_cloud->points[pointIdxNKNSearch[j]].x;
								average_y = average_y + original_cloud->points[pointIdxNKNSearch[j]].y;
								average_z = average_z + original_cloud->points[pointIdxNKNSearch[j]].z;

								average_r = average_r + original_cloud->points[pointIdxNKNSearch[j]].r;
								average_g = average_g + original_cloud->points[pointIdxNKNSearch[j]].g;
								average_b = average_b + original_cloud->points[pointIdxNKNSearch[j]].b;
							//}


						}

						interpolatedPoint.x = average_x / K;
						interpolatedPoint.y = average_y / K;
						interpolatedPoint.z = average_z / K;
						interpolatedPoint.r = average_r / K;
						interpolatedPoint.g = average_g / K;
						interpolatedPoint.b = average_b / K;

						add_point(interpolatedPoint);

						average_x = 0;
						average_y = 0;
						average_z = 0;

						average_r = 0;
						average_g = 0;
						average_b = 0;
					}

				}
				else
				{
 
					if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
						{
							if (pointNKNSquaredDistance[j] < current_dist)
							{
								current_dist = pointNKNSquaredDistance[j];
								average_x = original_cloud->points[pointIdxNKNSearch[j]].x;
								average_y = original_cloud->points[pointIdxNKNSearch[j]].y;
								average_z = original_cloud->points[pointIdxNKNSearch[j]].z;
							}
						}
						interpolatedPoint.x = average_x;
						interpolatedPoint.y = average_y;
						interpolatedPoint.z = average_z;
						interpolatedPoint.r = image_cloud->points[i].r;
						interpolatedPoint.g = image_cloud->points[i].g;
						interpolatedPoint.b = image_cloud->points[i].b;
						add_point(interpolatedPoint);
					}

				}

			}
		}
	}

	// this is the function that tries to smoothe the point cloud
	// we found that there was a problem by doing it this way that
	// elements such as light poles would turn into mini mountains 
	// due to the top of the lightpole (only a couple of points) 
	// being interpolated with close viewpoint points far away
	// speedwise to do something like this you have to use some kind of tree structure for the data 
	void Interpolate_the_current_3d_cloud
	(
	)
	{


		cout << "call to Interpolate_the_current_3d_cloud " << endl;


		//// Create a KD-Tree


		//pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_interpolated(new pcl::PointCloud<pcl::PointXYZ>);

		//copyPointCloud(*current_frame_pointcloud, *in_cloud);

		////pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
		//mls.setComputeNormals(true);
		//mls.setInputCloud(in_cloud);
		//mls.setSearchMethod(tree);
		//mls.setSearchRadius(0.03);
		//mls.setUpsamplingMethod( pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::UpsamplingMethod::SAMPLE_LOCAL_PLANE );
		//mls.setUpsamplingRadius(0.005);
		//mls.setUpsamplingStepSize(0.005);
		//mls.process(*cloud_new_interpolated);



		//pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_pre(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());

		copyPointCloud(*current_frame_pointcloud, *xyz_cloud);

		//for (int i = 0; i < xyz_cloud->points.size(); i++)
		//{
		//	cout << "some input " << xyz_cloud->points[i].x << endl;
		//}


		//fromPCLPointCloud2(*input, *xyz_cloud_pre);


		//// Filter the NaNs from the cloud
		//for (size_t i = 0; i < xyz_cloud_pre->size(); ++i)
		//	if (std::isfinite(xyz_cloud_pre->points[i].x))
		//		xyz_cloud->push_back(xyz_cloud_pre->points[i]);
		//xyz_cloud->header = xyz_cloud_pre->header;
		//xyz_cloud->height = 1;
		//xyz_cloud->width = static_cast<uint32_t> (xyz_cloud->size());
		//xyz_cloud->is_dense = false;

		float search_radius = 2;

		pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud_smoothed(new pcl::PointCloud<pcl::PointNormal>());

		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		mls.setInputCloud(xyz_cloud);
		mls.setSearchRadius(search_radius);
		//if (sqr_gauss_param_set) mls.setSqrGaussParam(sqr_gauss_param);
		//mls.setPolynomialOrder(polynomial_order);

		//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
		//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
		//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);

		mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
		mls.setPointDensity(100 * int(search_radius)); // 300 points in a 5 cm radius 
		mls.setUpsamplingRadius(0.050);
		mls.setUpsamplingStepSize(0.050);
		mls.setDilationIterations(1);
		mls.setDilationVoxelSize(0.1f);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		mls.setSearchMethod(tree);
		mls.setComputeNormals(false);
		mls.setPolynomialOrder(1);

		PCL_INFO("Computing smoothed surface and normals with search_radius %f , sqr_gaussian_param %f, polynomial order %d\n",
			mls.getSearchRadius(), mls.getSqrGaussParam(), mls.getPolynomialOrder());

		mls.process(*xyz_cloud_smoothed);




		//for (int i = 0; i < xyz_cloud_smoothed->points.size(); i++)
		//{
		//	cout << "some x " << xyz_cloud_smoothed->points[i].x << endl;
		//}






		//pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		//
		//kdtree.setInputCloud(current_frame_pointcloud);
		//
		//// for all of the pixels in the cloud 
		//for (int i = 0; i < current_frame_pointcloud->points.size(); i++)
		//{
		//	
		//	if (i % 100000 == 0)
		//	{
		//		cout << "interpolate " << i << endl;
		//	}
		//	pcl::PointXYZRGBA interpolatedPoint;
		//	// extract the point position of the pixel
		//	double x1 = current_frame_pointcloud->points[i].x;
		//	double y1 = current_frame_pointcloud->points[i].y;
		//	double z1 = current_frame_pointcloud->points[i].z;
		//	// set the pixel searchpoint 
		//	pcl::PointXYZRGBA searchPoint;
		//	searchPoint.x = x1;
		//	searchPoint.y = y1;
		//	searchPoint.z = z1;
		//	// set k and make lists for distances and indexes
		//	int K = 32;
		//	std::vector<int> pointIdxNKNSearch(K);
		//	std::vector<float> pointNKNSquaredDistance(K);
		//	double average_x = x1;
		//	double average_y = y1;
		//	double average_z = z1;
		//	double average_r = 255;
		//	double average_g = 255;
		//	double average_b = 255;
		//	float current_dist = 100000;
		//	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		//	{
		//		for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
		//		{
		//			
		//			average_x = average_x + current_frame_pointcloud->points[pointIdxNKNSearch[j]].x;
		//			average_y = average_y + current_frame_pointcloud->points[pointIdxNKNSearch[j]].y;
		//			average_z = average_z + current_frame_pointcloud->points[pointIdxNKNSearch[j]].z;
		//			average_r = average_r + current_frame_pointcloud->points[pointIdxNKNSearch[j]].r;
		//			average_g = average_g + current_frame_pointcloud->points[pointIdxNKNSearch[j]].g;
		//			average_b = average_b + current_frame_pointcloud->points[pointIdxNKNSearch[j]].b;
		//		}
		//		 average_r = 255;
		//		 average_g = 255;
		//		 average_b = 255;
		//		interpolatedPoint.x = average_x / K + 1;
		//		interpolatedPoint.y = average_y / K + 1;
		//		interpolatedPoint.z = average_z / K + 1;
		//		interpolatedPoint.r = average_r / K + 1;
		//		interpolatedPoint.g = average_g / K + 1;
		//		interpolatedPoint.b = average_b / K + 1;
		//		cloud_new_interpolated->points.push_back(interpolatedPoint);
		//		//add_point(interpolatedPoint);
		//		average_x = 0;
		//		average_y = 0;
		//		average_z = 0;
		//		average_r = 0;
		//		average_g = 0;
		//		average_b = 0;
		//	}
		//		
		//	
		//}

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_new_interpolated(new pcl::PointCloud<pcl::PointXYZRGBA>);

		copyPointCloud(*xyz_cloud_smoothed, *cloud_new_interpolated);

		current_frame_pointcloud = cloud_new_interpolated;

	}


};

class customframe
{
public:
	cv::Mat frame;
	customframe()
	{
		frame = cv::imread("E:\Trafik\tbd.jpeg", CV_LOAD_IMAGE_COLOR);
	}
	customframe(cv::Mat frame_in)
	{
		frame = frame_in;
	}
	void setframe(cv::Mat frame_in)
	{
		frame = frame_in;
	}
};

using namespace boost::filesystem;

class videodata
{
public:

	mutable boost::shared_mutex video_data_mutex; // the mutex to protect read and write cloud_Data;
	customframe current_frame_image;

	std::vector<customframe> image_data_frames;


	// innit the cloud data
	videodata()
	{

	}

	// allow other threads to read the data
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr read()
	{

	}

	// write the data 
	void write()
	{

	}


	void load_image_csv_testdata1(int amout)
	{
		//std::string path = "E:/Trafik/ImageData";
		//
		//// for all the files in the directory

		//int number = 1;




		//for (int i = 0; i < amout; i++)
		//{
		//	std::stringstream ss;
		//	ss << the_path;
		//	ss << "image data ";
		//	ss << std::to_string(number);
		//	ss << ")";
		//	ss << ".jpeg";

		//	std::cout << "reading:   " << ss.str() << std::endl;

		//	std::ifstream file(ss.str());


		//	number = number + 1;
		//}

		//image_data_frames.push_back( current_frame_pointcloud );





	}


	void save()
	{


	}


};