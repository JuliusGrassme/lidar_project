//#pragma once

bool enforceIntensitySimilarity(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
	float point_a_intensity = (point_a.r + point_a.g + point_a.b)*0.3333;
	float point_b_intensity = (point_b.r + point_b.g + point_b.b)*0.3333;

	if (std::abs(point_a_intensity - point_b_intensity) < 5.0f)
		return (true);
	else
		return (false);
}

bool enforceCurvatureOrIntensitySimilarity(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap();
	Eigen::Map<const Eigen::Vector3f> point_b_normal = point_b.getNormalVector3fMap();

	float point_a_intensity = (point_a.r + point_a.g + point_a.b)*0.3333;
	float point_b_intensity = (point_b.r + point_b.g + point_b.b)*0.3333;

	//if (std::abs(point_a_intensity - point_b_intensity) < 5.0f)
	//	return (true);
	if (std::abs(point_a_normal.dot(point_b_normal)) < 0.05)
		return (true);
	return (false);
}

bool customRegionGrowing(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();

	float point_a_intensity = (point_a.r + point_a.g + point_a.b)*0.3333;
	float point_b_intensity = (point_b.r + point_b.g + point_b.b)*0.3333;

	//cout << "squared_distance " << squared_distance << endl;
	if (std::abs(point_a_normal.dot(point_b_normal)) < (3.14 * 2 / 360) * 60)
		return (true);

	return (false);
}

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <iostream>
#include <thread>
#include <iostream>
#include <cstdint>
#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <pcl/common/transforms.h>
#include <windows.h>
#include <fstream>

using namespace cv;
using namespace std;

#include "VelodyneCapture.h"
#include "data_classes.h"





class interface
{
public:

	std::string video_window_title;

	int current_visualizer_state; //  it can be 1 signaling show and 0 signaling don't show

	int current_video_state; //  it can be 1 signaling show and 0 signaling don't show 

	int current_4_view_state; // if 1 show the 4 viewer

	int current_capture_state;

	int go_forward_frame_state; // signal to the viewer to go forward

	void interface_start()
	{
		video_window_title = "Camera frame";

		current_visualizer_state = 0;
		current_video_state = 0;
		current_capture_state = 0;
		current_4_view_state = 0;
		go_forward_frame_state = 0;

		while (true)
		{
			int command = wait_and_parse_the_input();

			excecuting_the_command(command);
		}

	};

	int get_current_visualizer_state()
	{
		return current_visualizer_state;
	};

	int get_current_video_state()
	{
		return current_video_state;
	};

	int get_current_capture_state()
	{
		return current_capture_state;
	};

	void set_current_visualizer_state(int input)
	{
		current_visualizer_state = input;
	};

	void set_current_video_state(int input)
	{
		current_video_state = input;
	};

	void set_current_capture_state(int input)
	{
		current_capture_state = input;
	};

	void set_current_4_view_state(int input)
	{
		current_4_view_state = input;
	}

	int get_current_4_view_state()
	{
		return current_4_view_state;
	}
	
	void set_go_forward_frame_state(int input)
	{
		go_forward_frame_state = input;
	}

	int get_go_forward_frame_state()
	{
		return go_forward_frame_state;
	}

	enum string_code
	{
		help,
		exit,
		restart,
		capture_all,
		stop_all,
		open_camera,
		close_camera,
		open_lidar,
		close_lidar,
		open_4vis,
		next_frame
	};

	int hashit(std::string const& inString)
	{
		if (inString == "help")         return help;
		if (inString == "exit")         return exit;
		if (inString == "restart")      return restart;
		if (inString == "capture_all")  return capture_all;
		if (inString == "capture_stop")  return capture_all;
		if (inString == "open_camera")  return open_camera;
		if (inString == "close_camera") return close_camera;
		if (inString == "open_lidar")   return open_lidar;
		if (inString == "close_lidar")  return close_lidar;
		if (inString == "4vis")  return open_4vis;
		if (inString == "next")  return next_frame;
	}

	int wait_and_parse_the_input()
	{

		while (true)
		{
			//Sleep(2500);
			//system("CLS");
			cout << "Waiting for a command" << endl;
			string input;
			getline(cin, input);

			switch (hashit(input.c_str()))
			{
			case help:
				return hashit(input.c_str());
				break;
			case exit:
				return hashit(input.c_str());
				break;
			case restart:
				return hashit(input.c_str());
				break;
			case capture_all:
				return hashit(input.c_str());
				break;
			case stop_all:
				return hashit(input.c_str());
				break;
			case open_camera:
				return hashit(input.c_str());
				break;
			case close_camera:
				return hashit(input.c_str());
				break;
			case open_lidar:
				return hashit(input.c_str());
				break;
			case close_lidar:
				return hashit(input.c_str());
				break;
			case open_4vis:
				return hashit(input.c_str());
				break;
			case next_frame:
				return hashit(input.c_str());
				break;
			default:
				cout << "Invalid command" << endl;
				break;
			}

		}

	};

	void excecuting_the_command(int command)
	{
		cout << "excecuting command " << command << endl;

		switch (command)
		{
		case help:
			command_help();
			break;
		case exit:
			command_exit();
			break;
		case restart:
			command_restart();
			break;
		case capture_all:
			command_capture_all();
			break;
		case stop_all:
			command_stop_all();
			break;
		case open_camera:
			command_open_camera();
			break;
		case close_camera:
			command_close_camera();
			break;
		case open_lidar:
			command_open_lidar();
			break;
		case close_lidar:
			command_close_lidar();
			break;
		case open_4vis:
			return command_open_4_view();
			break;
		case next_frame:
			return command_next_frame();
			break;
		}

	};

	void command_help()
	{
		cout << "help        " << endl;
		cout << "help        " << endl;
		cout << "exit        " << endl;
		cout << "restart     " << endl;
		cout << "capture_all " << endl;
		cout << "open_camera " << endl;
		cout << "close_camera" << endl;
		cout << "open_lidar  " << endl;
		cout << "close_lidar " << endl;
	};

	void command_exit()
	{
		cout << "command_exit command" << endl;
		cout << "NOT IMPLEMENTED YET" << endl;
	};

	void command_restart()
	{
		cout << "command_restart command" << endl;
		cout << "NOT IMPLEMENTED YET" << endl;
	};

	void command_capture_all()
	{
		cout << "command_capture_all command" << endl;
		set_current_capture_state(1);
	};

	void command_stop_all()
	{
		cout << "command_stop_capture_all command" << endl;
		set_current_capture_state(0);
	};

	void command_open_camera()
	{
		cout << "command_open_camera command" << endl;
		set_current_video_state(1);
	};

	void command_close_camera()
	{
		cout << "command_close_camera command" << endl;
		set_current_video_state(0);
	};

	void command_open_lidar()
	{
		cout << "command_open_lidar command" << endl;
		set_current_visualizer_state(1);
	};

	void command_close_lidar()
	{
		cout << "command_close_lidar command" << endl;
		set_current_visualizer_state(0);
	};

	void command_next_frame()
	{
		cout << "next frame command" << endl;
		set_go_forward_frame_state(1);
	};

	void command_open_4_view()
	{
		cout << "command_open_4_view command" << endl;
		set_current_4_view_state(1);
	};

	void command_close_4_view()
	{
		cout << "command_close_4_view command" << endl;
		set_current_4_view_state(0);
	};

};



//auto start_time = std::chrono::high_resolution_clock::now();
//std::cout << "stringstream  " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() << std::endl;
// this class handels the networking with the lidar
// it also handels syncronization between a lidar frame(a full rotation)
// and when the camera takes a picture
class networking // manager thread 
{

public:
	int networking_state;
	boost::mutex networking_mutex;
	boost::thread* networking_thread;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr networking_cloud;
	int* sync_pointer;
	int* sync_cloud;

	void networking_start(interface* the_interface, clouddata* cloud_Data_ptr, int* shared_value_for_sync, int* pointer_for_signaling_cloud_vizualizertiming)
	{
		sync_pointer = shared_value_for_sync;
		sync_cloud = pointer_for_signaling_cloud_vizualizertiming;
		networking_state = 1;

		networking_thread = new boost::thread(boost::bind(&networking::networking_recive, this, cloud_Data_ptr));
	};

	void networking_recive(clouddata* cloud_Data_ptr)
	{
		while (networking_state == 1)
		{

			//#pragma region
			//cout << "networking waiting for widget to want a time capture" << endl;
			////while(the_gui_interface->get_current_time_capture_state() == 0)
			////{
			////	
			////}
			//cout << "widget to want a time capture" << endl;
			//#pragma endregion wait for the widget to want a time capture started

			#pragma region	
			const boost::asio::ip::address address = boost::asio::ip::address::from_string("192.168.1.201");
			const unsigned short port = 2368;

			velodyne::HDL32ECapture capture(address, port);
			//velodyne::HDL32ECapture capture("E:\Trafik\2019-07-08-13-07-31_Velodyne-HDL-32-Data.pcap");


			if (!capture.isOpen()) // this does not block in case of the lidar no being connected 
			{
				std::cout << "Can not access Lidar" << std::endl;
			}
			else
			{
				std::cout << "Access to lidar aquired" << std::endl;
			}
			#pragma endregion Start capturing thread for lidar data


			//std::vector<custompoint> new_points_vector;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

			while (capture.isRun() && networking_state == 1)
			{

				auto network_time = std::chrono::high_resolution_clock::now();

				#pragma region
				std::vector<velodyne::Laser> lasers;
				capture >> lasers;

				if (lasers.empty())
				{
					continue;
				}
				#pragma endregion Wait while the lidar is collecting a full rotation of data

				#pragma region
				//int full_rotation_reached = 0;
				std::sort(lasers.begin(), lasers.end());
				for (const velodyne::Laser& laser : lasers) // Access to Laser Data
				{
					const double distance = static_cast<double>(laser.distance)*0.001;
					const double azimuth = laser.azimuth  * 3.1415 / 180.0;
					const double vertical = laser.vertical * 3.1415 / 180.0;
					float xp = static_cast<float>((distance * std::cos(vertical)) * std::sin(azimuth));
					float yp = static_cast<float>((distance * std::cos(vertical)) * std::cos(azimuth));
					float zp = static_cast<float>((distance * std::sin(vertical)));
					unsigned char rp = laser.intensity;

					pcl::PointXYZRGBA basic_point;
					basic_point.x = xp;
					basic_point.y = yp;
					basic_point.z = zp;
					basic_point.a = rp;
					new_points_cloud_ptr->points.push_back(basic_point);

					//new_points_vector.push_back( custompoint(xp, yp, zp, rp) );

				}
				#pragma endregion Add all the data form the lidar lasers to cloud

				#pragma region
				cloud_Data_ptr->write(new_points_cloud_ptr);
				#pragma endregion Save the cloud data into the data container 


				#pragma region
				*sync_pointer = 1;
				*sync_cloud = 1;
				#pragma endregion Signal the other threads to start saving data

				//std::cout << "network time " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - network_time).count() << std::endl;

				auto wait_time = std::chrono::high_resolution_clock::now();

				#pragma region
				while (*sync_pointer == 1 || *sync_cloud == 1)
				{
					std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::nanoseconds(10000));
				}
				#pragma endregion Wait for other threads to finish and be ready 

				//#pragma region 
				//if ( the_gui_interface->get_current_time_capture_state() == 0)
				//{
				//	// signal the camera and vizualizer thread to stop
				//	cout << "widget wants the capture to stop" << endl;
				//	break;
				//	
				//}
				//#pragma endregion if the widget wants to stop capturing break the current capture round

				//std::cout << "wait time " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - wait_time).count() << std::endl;

			}

		}

	};

	void networking_stop()
	{
		networking_state = 0;
		networking_thread->join();
	};

};


// this class handels the networking with the axis camera
// it also handels the saving of the camera data 
class video
{

public:

	interface* the_interface_video;
	int video_state;
	int* shared_value_for_syncronization;
	boost::thread* video_thread;
	int image_number;

	int video_debug;

	void video_start(interface* the_interface, int* shared_value_for_syncronization_input)
	{
		shared_value_for_syncronization = shared_value_for_syncronization_input;

		video_state = 1;

		image_number = 1;

		video_thread = new boost::thread(boost::bind(&video::GetVideoImage, this));

		the_interface_video = the_interface;

	};

	void GetVideoImage()
	{
#pragma region
		cv::VideoCapture cap = cv::VideoCapture("rtsp://root:admin@192.168.0.90/axis-media/media.amp");
		auto program_start_time = std::chrono::high_resolution_clock::now();
#pragma endregion setup capture 

#pragma region
		VideoWriter outputVideo;
		int width = 1920;
		int height = 1080;
		Size S = Size(width, height);



		const string filename = SAVEPATH + "out_vid.avi";
		const string logpath = SAVEPATH + "log_file.txt";
		int fps = 20;
		// to use this you have to have the right encoder to get x264 use...
		// http://www.fourcc.org/downloads/ffdshow-mpeg-4-video-25/

		int fourcc = CV_FOURCC('X', '2', '6', '4');
		// open video file on disk

		outputVideo.open(filename, fourcc, fps, S);

#pragma endregion setup write 

		std::ofstream log_file(logpath, std::ios_base::out | std::ios_base::trunc);

#pragma region
		while (video_state == 1)
		{

			//while (the_gui_interface->get_current_video_state() == 0) // wait untill the user wants the data camera
			//{
			//}

#pragma region
			if (*shared_value_for_syncronization == 1) // || the_gui_interface->get_current_visualizer_state() == 1
			{
				//auto video_time = std::chrono::high_resolution_clock::now();

#pragma region
				cv::Mat src;
				cap >> src;
#pragma endregion get image from the video camera

#pragma region
				if (the_interface_video->get_current_video_state() == 1)
				{
					cv::imshow(the_interface_video->video_window_title, src);
					waitKey(20);
				}
#pragma endregion if the command interface want a window shown containing the image stream

#pragma region
				if (the_interface_video->get_current_video_state() == 0 && cvGetWindowHandle(the_interface_video->video_window_title.c_str()))
				{
					cv::destroyWindow(the_interface_video->video_window_title);

				}
#pragma endregion if the command interface wants the window destroyes kill the window with the video


#pragma region 
				if (the_interface_video->get_current_capture_state() == 1)
				{

					if (!outputVideo.isOpened())
					{
						std::cout << "outputvideo not open" << std::endl;
						//outputVideo.open(filename, fourcc, fps, S);
					}


					if (!src.empty())// Check for invalid input
					{
						//std::cout << "Could find image" << std::endl;
					}
					else
					{
						std::cout << "Could not open or find the image" << std::endl;
					}

					outputVideo.write(src);

					//std::cout << "Video write: " << std::endl;

					std::stringstream ssx;
					ssx << boost::posix_time::microsec_clock::local_time();
					std::string times = ssx.str();
					log_file << image_number << " " << times << std::endl;
					image_number = image_number + 1;

				}
#pragma endregion write image to video


#pragma region
				*shared_value_for_syncronization = 0;
#pragma endregion signal done

				//std::cout << "video time " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - video_time).count() << std::endl;

			}
#pragma endregion if it is time yet to take a picture


		}
#pragma endregion while the thread is suppose to run

		outputVideo.release();

	}

	void video_stop()
	{
		//std::cout << "ServerVideoStream Shutdown" << std::endl;
		video_state = 0;
	}

};


//
class the_tracker
{
public:
	
	ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker_;

	void tracker_start()
	{

		#pragma region setup of the filter 

		KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker(4);
		ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker_;

		tracker.setMaximumParticleNum(1000);
		tracker.setDelta(0.99);

		tracker.setEpsilon(0.2);
		ParticleXYZRPY bin_size;
		bin_size.x = 0.1f;
		bin_size.y = 0.1f;
		bin_size.z = 0.1f;
		bin_size.roll = 0.1f;
		bin_size.pitch = 0.1f;
		bin_size.yaw = 0.1f;
		tracker.setBinSize(bin_size);

		//Set all parameters for  ParticleFilter
		
		tracker_.setTrans(Eigen::Affine3f::Identity());

		tracker_ = tracker;

		std::vector<double> default_step_covariance = std::vector<double>(6, 0.15 * 0.15);
		default_step_covariance[3] *= 40.0;
		default_step_covariance[4] *= 40.0;
		default_step_covariance[5] *= 40.0;
		tracker_.setStepNoiseCovariance(default_step_covariance);

		std::vector<double> initial_noise_covariance = std::vector<double>(6, 1.0);
		tracker_.setInitialNoiseCovariance(initial_noise_covariance);

		std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
		tracker_.setInitialNoiseMean(default_initial_mean);
		tracker_.setIterationNum(10);
		tracker_.setParticleNum(600);
		tracker_.setResampleLikelihoodThr(0.00);
		tracker_.setUseNormal(false);


		//Setup coherence object for tracking
		ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr coherence(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>);

		DistanceCoherence<pcl::PointXYZRGBA>::Ptr distance_coherence(new DistanceCoherence<pcl::PointXYZRGBA>);
		coherence->addPointCoherence(distance_coherence);

		pcl::search::Octree<pcl::PointXYZRGBA>::Ptr search(new pcl::search::Octree<pcl::PointXYZRGBA>(0.01));
		coherence->setSearchMethod(search);
		coherence->setMaximumDistance(10);

		tracker_.setCloudCoherence(coherence);

		#pragma endregion

	}


};


//Filter along a specified dimension
void filterPassThrough(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 10.0);
	pass.setKeepOrganized(false);
	pass.setInputCloud(cloud);
	pass.filter(result);
}


//
void gridSampleApprox(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result, double leaf_size)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
}


// this class handels vizualizing 4 windows in a single viewport the cloud data when you call that 
class scene_visualizer
{

public:
	interface* the_interface_scene_visualizer;
	int scene_visualizer_state;
	boost::thread* scene_visualizer_thread;
	int* pointer_signaling_cloud_timing;
	int data_number;
	int viewer_excists;
	double image_time;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	std::vector<the_tracker> my_list_trackers;
	


	//boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
	//{
	//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//	viewer->setBackgroundColor(0, 0, 0);
	//	pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
	//	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgba, "sample cloud");
	//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//	viewer->addCoordinateSystem(1.0);
	//	viewer->initCameraParameters();
	//	return viewer;
	//}

	// this function returns the viewer which is used to display the data.
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> fourvis
	//(
	//	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_1,
	//	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_2,
	//	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_3,
	//	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_4/*,
	//	pcl::PointCloud<pcl::Normal>::ConstPtr normals_1,
	//	pcl::PointCloud<pcl::Normal>::ConstPtr normals_2,
	//	pcl::PointCloud<pcl::Normal>::ConstPtr normals_3,
	//	pcl::PointCloud<pcl::Normal>::ConstPtr normals_4*/
	//)



	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
		void* viewer_void)
	{
		pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
		if (event.getKeySym() == "r" && event.keyDown())
		{
			std::cout << "r was pressed => removing all text" << std::endl;
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis
	(
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_1,
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_2,
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_3,
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_4
	)
	{
		// Open 3D viewer and add point cloud and normals
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("4 View 3D Viewer"));
		//viewer->setShowFPS(true);
		// createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport);
	
		int v1 = 0;
		viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
		viewer->addText("Viewport_1", 10, 10, "v1_text_id", v1);
		pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb_1(cloud_1);
		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_1, rgb_1, "cloud_1_id", v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_1_id");
		//viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud_1, normals_1, 10, 0.05, "normals_1_id", v1);
	
		int v2 = 1;
		viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
		viewer->addText("Viewport_2", 10, 10, "v2_text_id", v2);
		pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb_2(cloud_2);
		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_2, rgb_2, "cloud_2_id", v2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_2_id");
		//viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud_1, normals_1, 10, 0.05, "normals_2_id", v2);
	
		int v3 = 2;
		viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
		viewer->setBackgroundColor(0.0, 0.0, 0.0, v3);
		viewer->addText("Viewport_3", 10, 10, "v3_text_id", v3);
		pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb_3(cloud_3);
		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_3, rgb_3, "cloud_3_id", v3);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_3_id");
		//viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud_1, normals_1, 10, 0.05, "normals_3_id", v3);
	
		int v4 = 3;
		viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
		viewer->setBackgroundColor(0.0, 0.0, 0.0, v4);
		viewer->addText("Viewport_4", 10, 10, "v4_text_id", v4);
		pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb_4(cloud_4);
		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_4, rgb_4, "cloud_4_id", v4);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_4_id");
		//viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud_1, normals_1, 10, 0.05, "normals_4_id", v4);
			
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
	
		return viewer;
	};

	void scene_visualizer_start(interface* the_interface, clouddata* cloud_Data_ptr1, clouddata* cloud_Data_ptr2, clouddata* cloud_Data_ptr3, clouddata* cloud_Data_ptr4, int* pointer_for_signaling_cloud_vizualizertiming)
	{
		image_time = 1468.000;
		viewer_excists = 0;
		the_interface_scene_visualizer = the_interface;
		pointer_signaling_cloud_timing = pointer_for_signaling_cloud_vizualizertiming;
		scene_visualizer_state = 1;
		data_number = 1;
		scene_visualizer_thread = new boost::thread(boost::bind(&scene_visualizer::scene_visualizer_render, this, cloud_Data_ptr1, cloud_Data_ptr2, cloud_Data_ptr3, cloud_Data_ptr4 ));
	};

	void scene_visualizer_render(clouddata* cloud_Data_ptr1, clouddata* cloud_Data_ptr2, clouddata* cloud_Data_ptr3, clouddata* cloud_Data_ptr4)
	{
		while (scene_visualizer_state == 1) // while the thread is not shutting down
		{

			// if the command interface indicates that it is time to go forward 1 frame
			// this is done by calling the next command from the comand interface

			if (the_interface_scene_visualizer->get_go_forward_frame_state() == 1)
			{

				#pragma region old version of the window update code
				//
				//// descide of a new time aka current time + 0.5
				//image_time = image_time + 0.500;
				////update the data 

				//cout << "skipping to the new frame  :  " << image_time << endl;

				//cloud_Data_ptr1->current_frame_pointcloud->points.clear();
				//cloud_Data_ptr2->current_frame_pointcloud->points.clear();
				//cloud_Data_ptr3->current_frame_pointcloud->points.clear();
				//cloud_Data_ptr4->current_frame_pointcloud->points.clear();

				//float image_timeing = cloud_Data_ptr1->load_jpg_image_pointcloud(image_time);

				//cout << "image_timeing : " << image_timeing << endl;

				//image_timeing = image_timeing + 2.15; // 2.1

				//float cloud1_timeing = cloud_Data_ptr2->load_csv_cloud_pointcloud(image_timeing, false, false); //load without projection
				//float cloud2_timeing = cloud_Data_ptr3->load_csv_cloud_pointcloud(image_timeing, true, true); // load with projection

				//cout << "cloud_timeing : " << cloud1_timeing << endl;

				//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image_cloud = cloud_Data_ptr1->read();
				//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud = cloud_Data_ptr2->read();
				//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projected_cloud = cloud_Data_ptr3->read();

				////the_clouddata_view_4.load_csv_cloud_pointcloud(image_time, false);
				//cloud_Data_ptr4->load_the_fused_point_clouds(image_cloud, original_cloud, projected_cloud, true);

				//the_interface_scene_visualizer->set_go_forward_frame_state(0);
				//
				#pragma endregion


				#pragma region newer version where tracking is not quite working yet


				#pragma region load pointcloud at timestep t

				//load the nearest image to the given time 
				float image_timeing_ = cloud_Data_ptr1->load_jpg_image_pointcloud(image_time);
				// offset as to syncronize the
				//data between the 2
				image_timeing_ = image_timeing_ + 2.15;
				// load the pointcloud nearest to the time the image was taken 
				float cloud1_timeing_ = cloud_Data_ptr2->load_csv_cloud_pointcloud(image_timeing_, false, false); //load without projection
				// subtract the background of that frame
				cloud_Data_ptr2->subtract_background("C:/testdir/background.csv");
				// also load the pointcloud with projection for the fusion of images and point cloud 
				//float cloud2_timeing = cloud_Data_ptr3->load_csv_cloud_pointcloud(image_timeing, true, true); // load with projection

				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::copyPointCloud(*cloud_Data_ptr2->current_frame_pointcloud, *cloud_in_);

				#pragma endregion


				#pragma region load pointcloud at timestep t+1
				// get the next image time, here i'm skipping 250ms as to get a new image 
				image_time = image_time + 0.250;
				//load the nearest image to the given time 
				float image_timeing = cloud_Data_ptr1->load_jpg_image_pointcloud(image_time);
				// offset as to syncronize the data between the 2
				image_timeing = image_timeing + 2.15;
				// load the pointcloud nearest to the time the image was taken 
				float cloud1_timeing = cloud_Data_ptr2->load_csv_cloud_pointcloud(image_timeing, false, false); //load without projection
				// subtract the background of that frame
				cloud_Data_ptr2->subtract_background("C:/testdir/background.csv");
				// also load the pointcloud with projection for the fusion of images and point cloud 
				//float cloud2_timeing = cloud_Data_ptr3->load_csv_cloud_pointcloud(image_timeing, true, true); // load with projection

				// data container for in data 
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in_plus_one(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::copyPointCloud(*cloud_Data_ptr2->current_frame_pointcloud, *cloud_in_plus_one);

				#pragma endregion


				#pragma region identifying clusters of normal size in the old frame

				// compute the normals of the pointcloud using kdtree as search tree
				pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr search_tree_(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::copyPointCloud(*cloud_in_, *cloud_with_normals_);
				pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne_;
				ne_.setInputCloud(cloud_in_);
				ne_.setSearchMethod(search_tree_);
				ne_.setRadiusSearch(5);
				ne_.compute(*cloud_with_normals_);

				// do the clustering and sort outputs into 3 classes 
				pcl::IndicesClustersPtr clusters_old(new pcl::IndicesClusters);
				pcl::IndicesClustersPtr small_clusters_old(new pcl::IndicesClusters);
				pcl::IndicesClustersPtr large_clusters_old(new pcl::IndicesClusters);
				pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec_(true);
				cec_.setInputCloud(cloud_with_normals_);
				cec_.setConditionFunction(&customRegionGrowing);
				cec_.setClusterTolerance(0.8);
				cec_.setMinClusterSize(25);
				cec_.setMaxClusterSize(cloud_with_normals_->points.size());
				cec_.segment(*clusters_old);
				cec_.getRemovedClusters(small_clusters_old, large_clusters_old);


				cout << "small clusters size: " << small_clusters_old->size() << endl;
				// Using the intensity channel for lazy visualization of the output
				//for (int i = 0; i < small_clusters_old->size(); ++i)
				//{
				//	for (int j = 0; j < (*small_clusters_old)[i].indices.size(); ++j)
				//	{
				//		int lr = rand() % 256;
				//		int lg = rand() % 256;
				//		int lb = rand() % 256;

				//		cloud_in_->points[(*small_clusters_old)[i].indices[j]].r = lr;
				//		cloud_in_->points[(*small_clusters_old)[i].indices[j]].g = 0;
				//		cloud_in_->points[(*small_clusters_old)[i].indices[j]].b = 0;
				//	}
				//}

				cout << "large clusters size: " << large_clusters_old->size() << endl;
				//for (int i = 0; i < large_clusters_old->size(); ++i)
				//{
				//	for (int j = 0; j < (*large_clusters_old)[i].indices.size(); ++j)
				//	{
				//		cloud_in_->points[(*large_clusters_old)[i].indices[j]].r = 0;
				//		cloud_in_->points[(*large_clusters_old)[i].indices[j]].g = 0;
				//		cloud_in_->points[(*large_clusters_old)[i].indices[j]].b = 255;
				//	}
				//}

				cout << "normal clusters size: " << clusters_old->size() << endl;
				for (int i = 0; i < clusters_old->size(); ++i)
				{
					int label = (rand() % 128) + 128;
					for (int j = 0; j < (*clusters_old)[i].indices.size(); ++j)
					{
						cloud_in_->points[(*clusters_old)[i].indices[j]].r = label;
						cloud_in_->points[(*clusters_old)[i].indices[j]].g = label;
						cloud_in_->points[(*clusters_old)[i].indices[j]].b = label;
					}
				}
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr the_point_cloud_with_all_the_clusters_in_old_frame(new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::copyPointCloud(*cloud_in_, *the_point_cloud_with_all_the_clusters_in_old_frame);

				//cloud_Data_ptr4->current_frame_pointcloud = the_point_cloud_with_all_the_clusters_in;



				#pragma endregion 


				#pragma region identifying clusters of normal size in the new frame



				// compute the normals of the pointcloud using kdtree as search tree
				pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				pcl::copyPointCloud(*cloud_in_plus_one, *cloud_with_normals);
				pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
				ne.setInputCloud(cloud_in_plus_one);
				ne.setSearchMethod(search_tree);
				ne.setRadiusSearch(5);
				ne.compute(*cloud_with_normals);

				// do the clustering and sort outputs into 3 classes 
				pcl::IndicesClustersPtr clusters_new(new pcl::IndicesClusters);
				pcl::IndicesClustersPtr small_clusters_new(new pcl::IndicesClusters);
				pcl::IndicesClustersPtr large_clusters_new(new pcl::IndicesClusters);
				pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec(true);
				cec.setInputCloud(cloud_with_normals);
				cec.setConditionFunction(&customRegionGrowing);
				cec.setClusterTolerance(0.8);
				cec.setMinClusterSize(25);
				cec.setMaxClusterSize(cloud_with_normals->points.size());
				cec.segment(*clusters_new);
				cec.getRemovedClusters(small_clusters_new, large_clusters_new);


				cout << "small clusters size: " << small_clusters_new->size() << endl;
				// Using the intensity channel for lazy visualization of the output
				//for (int i = 0; i < small_clusters_new->size(); ++i)
				//{
				//	for (int j = 0; j < (*small_clusters_new)[i].indices.size(); ++j)
				//	{
				//		int lr = rand() % 256;
				//		int lg = rand() % 256;
				//		int lb = rand() % 256;

				//		cloud_in_plus_one->points[(*small_clusters_new)[i].indices[j]].r = lr;
				//		cloud_in_plus_one->points[(*small_clusters_new)[i].indices[j]].g = 0;
				//		cloud_in_plus_one->points[(*small_clusters_new)[i].indices[j]].b = 0;
				//	}
				//}

				cout << "large clusters size: " << large_clusters_new->size() << endl;
				//for (int i = 0; i < large_clusters_new->size(); ++i)
				//{
				//	for (int j = 0; j < (*large_clusters_new)[i].indices.size(); ++j)
				//	{
				//		cloud_in_plus_one->points[(*large_clusters_new)[i].indices[j]].r = 0;
				//		cloud_in_plus_one->points[(*large_clusters_new)[i].indices[j]].g = 0;
				//		cloud_in_plus_one->points[(*large_clusters_new)[i].indices[j]].b = 255;
				//	}
				//}

				cout << "normal clusters size: " << clusters_new->size() << endl;
				for (int i = 0; i < clusters_new->size(); ++i)
				{
					int label = (rand() % 128) + 128;
					for (int j = 0; j < (*clusters_new)[i].indices.size(); ++j)
					{
						cloud_in_plus_one->points[(*clusters_new)[i].indices[j]].r = label;
						cloud_in_plus_one->points[(*clusters_new)[i].indices[j]].g = label;
						cloud_in_plus_one->points[(*clusters_new)[i].indices[j]].b = label;
					}
				}

				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr the_point_cloud_with_all_the_clusters_in_new_frame(new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::copyPointCloud(*cloud_in_plus_one, *the_point_cloud_with_all_the_clusters_in_new_frame);




				#pragma endregion 


				#pragma region for every cluster centerpoint in last frame

				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr particles(new pcl::PointCloud< pcl::PointXYZRGBA >);

				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr all_of_the_refrance_cloud_old(new pcl::PointCloud<pcl::PointXYZRGBA>);

				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr all_of_the_result_clouds(new pcl::PointCloud<pcl::PointXYZRGBA>);

				// for 
				for (int i = 0; i < clusters_old->size(); ++i)
				{

					//extract the pointcloud from the old frame // this will be the refrance pointcloud
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr the_refrance_cloud_old(new pcl::PointCloud<pcl::PointXYZRGBA>);
					the_refrance_cloud_old->points.clear();

					int labelr = (rand() % 255 - 32) + 32;
					int labelg = (rand() % 255 - 32) + 32;
					int labelb = (rand() % 255 - 32) + 32;

					for (int j = 0; j < (*clusters_old)[i].indices.size(); ++j)
					{
						pcl::PointXYZRGBA basic_point;
						basic_point.x = the_point_cloud_with_all_the_clusters_in_old_frame->points[(*clusters_old)[i].indices[j]].x;
						basic_point.y = the_point_cloud_with_all_the_clusters_in_old_frame->points[(*clusters_old)[i].indices[j]].y;
						basic_point.z = the_point_cloud_with_all_the_clusters_in_old_frame->points[(*clusters_old)[i].indices[j]].z;

						basic_point.r = labelr;
						basic_point.g = labelg;
						basic_point.b = labelb;

						basic_point.a = the_point_cloud_with_all_the_clusters_in_old_frame->points[(*clusters_old)[i].indices[j]].a;
						the_refrance_cloud_old->points.push_back(basic_point);
						all_of_the_refrance_cloud_old->points.push_back(basic_point);
					}


					#pragma region setup of the filter 

					KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker(4);
					ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker_;

					tracker.setMaximumParticleNum(1000);
					tracker.setDelta(0.99);

					tracker.setEpsilon(0.2);
					ParticleXYZRPY bin_size;
					bin_size.x = 0.1f;
					bin_size.y = 0.1f;
					bin_size.z = 0.1f;
					bin_size.roll = 0.1f;
					bin_size.pitch = 0.1f;
					bin_size.yaw = 0.1f;
					tracker.setBinSize(bin_size);

					//Set all parameters for  ParticleFilter

					tracker_.setTrans(Eigen::Affine3f::Identity());

					tracker_ = tracker;

					std::vector<double> default_step_covariance = std::vector<double>(6, 0.15 * 0.15);
					default_step_covariance[3] *= 4.0;
					default_step_covariance[4] *= 4.0;
					default_step_covariance[5] *= 4.0;
					tracker_.setStepNoiseCovariance(default_step_covariance);

					std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.1);
					tracker_.setInitialNoiseCovariance(initial_noise_covariance);

					std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
					tracker_.setInitialNoiseMean(default_initial_mean);
					tracker_.setIterationNum(10);
					tracker_.setParticleNum(600);
					tracker_.setResampleLikelihoodThr(0.00);
					tracker_.setUseNormal(false);


					//Setup coherence object for tracking
					ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr coherence(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>);

					DistanceCoherence<pcl::PointXYZRGBA>::Ptr distance_coherence(new DistanceCoherence<pcl::PointXYZRGBA>);
					coherence->addPointCoherence(distance_coherence);

					pcl::search::Octree<pcl::PointXYZRGBA>::Ptr search(new pcl::search::Octree<pcl::PointXYZRGBA>(0.01));
					coherence->setSearchMethod(search);
					coherence->setMaximumDistance(5);

					tracker_.setCloudCoherence(coherence);

					#pragma endregion

					#pragma region set the input of the filter

					//prepare the model of the target that the tracker is meant to track

					// compute the centroid of the object to be tracked
					Eigen::Vector4f c;
					pcl::compute3DCentroid<pcl::PointXYZRGBA>(*the_refrance_cloud_old, c);

					// create some afine transform
					Eigen::Affine3f trans = Eigen::Affine3f::Identity();
					trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);

					// transform the cloud using that transform
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref(new pcl::PointCloud<pcl::PointXYZRGBA>);
					pcl::transformPointCloud<pcl::PointXYZRGBA>(*the_refrance_cloud_old, *transed_ref, trans.inverse());

					//// the downsampling, found to increase fps, so thats why its here
					//double downsampling_grid_size_ = 0.002;
					//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref_downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
					//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);


					//set reference model and transformation
					tracker_.setReferenceCloud(transed_ref);//transed_ref_downsampled);
					tracker_.setTrans(trans);

					#pragma endregion

					#pragma region run a cloud through the filter set the 3 viewport as its output

					tracker_.setInputCloud(the_point_cloud_with_all_the_clusters_in_new_frame);
					tracker_.compute();


					auto temp_particales = tracker_.getParticles();

					ParticleXYZRPY temp_result = tracker_.getResult();

					for (int s = 0; s < temp_particales->points.size(); s++)
					{
						pcl::PointXYZRGBA basic_point;
						basic_point.x = temp_particales->points[s].x;
						basic_point.y = temp_particales->points[s].y;
						basic_point.z = temp_particales->points[s].z;
						basic_point.r = the_refrance_cloud_old->points[0].r;
						basic_point.g = the_refrance_cloud_old->points[0].g;
						basic_point.b = the_refrance_cloud_old->points[0].b;
						basic_point.a = 255;
						particles->points.push_back(basic_point);
						the_point_cloud_with_all_the_clusters_in_new_frame->points.push_back(basic_point);
					}



					Eigen::Affine3f transformation = tracker_.toEigenMatrix(temp_result);

					//cout << transformation.matrix() << endl;

					//move close to camera a little for better visualization
					//transformation.translation() += Eigen::Vector3f(0.0f, 0.0f, -0.005f);

					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
					pcl::transformPointCloud<pcl::PointXYZRGBA>(*(tracker_.getReferenceCloud()), *result_cloud, transformation);

					for (int i = 0; i < result_cloud->points.size(); i++)
					{
						all_of_the_result_clouds->points.push_back(result_cloud->points[i]);
					}

					//pcl::PointXYZRGBA basic_point2; // this is the singular point the filter thinks there is a cluster at 
					//basic_point2.x = temp_result.x;
					//basic_point2.y = temp_result.y;
					//basic_point2.z = temp_result.z;
					//basic_point2.r = the_refrance_cloud_old->points[0].r;
					//basic_point2.g = the_refrance_cloud_old->points[0].g;
					//basic_point2.b = the_refrance_cloud_old->points[0].b;
					//basic_point2.a = 255;
					//all_of_the_result_clouds->points.push_back(basic_point2);





					//for (int i = 0; i < 500; i++)
					//{
					//	float randx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 0.5)) - static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 0.5));
					//	float randy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 0.5)) - static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 0.5));
					//	float randz = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 0.5)) - static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 0.5));

					//	//cout << "x " << randx << endl;
					//	//cout << "y " << randy << endl;
					//	//cout << "z " << randz << endl;


					//	pcl::PointXYZRGBA basic_point2; // this is the singular point the filter thinks there is a cluster at 
					//	basic_point2.x = temp_result.x + randx;
					//	basic_point2.y = temp_result.y + randy;
					//	basic_point2.z = temp_result.z + randz;
					//	basic_point2.r = the_refrance_cloud_old->points[0].r;;
					//	basic_point2.g = the_refrance_cloud_old->points[0].g;
					//	basic_point2.b = the_refrance_cloud_old->points[0].b;
					//	basic_point2.a = 255;
					//	the_point_cloud_with_all_the_clusters_in_new_frame->points.push_back(basic_point2);
					//}

					#pragma endregion

				}
				#pragma endregion 


				#pragma region fill relevant vis frame with data

				float cloud1_timeingdsa = cloud_Data_ptr1->load_csv_cloud_pointcloud(image_timeing, false, false); // new pointcloud
				cloud_Data_ptr2->current_frame_pointcloud = all_of_the_refrance_cloud_old;                         // the clusters after background subtraction 
				cloud_Data_ptr3->current_frame_pointcloud = the_point_cloud_with_all_the_clusters_in_new_frame;    // marking in the new pointcloud where the 
				cloud_Data_ptr4->current_frame_pointcloud = all_of_the_result_clouds;                              // clusters in the old frame with colors
				#pragma endregion
				
				
				#pragma endregion


				// signal to the vizualizer that the process of gowing forward 1 frame have been compleated 
				the_interface_scene_visualizer->set_go_forward_frame_state(0);


			}


			// if the viewer does not excist create it 
			if (viewer_excists == 0 && the_interface_scene_visualizer->get_current_4_view_state() == 1)
			{
				viewer = customColourVis(cloud_Data_ptr1->read(), cloud_Data_ptr2->read(), cloud_Data_ptr3->read(), cloud_Data_ptr4->read() );
				viewer_excists = 1;

			}

			// if the viewer excists and the interface wants it to excist update the pointclouds
			// actually this should proberly just be viewer->updatePointCloud instead of remove / add
			if (viewer_excists == 1 && the_interface_scene_visualizer->get_current_4_view_state() == 1)
			{
				pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba1(cloud_Data_ptr1->read());
				viewer->removePointCloud("cloud_1_id");
				viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_Data_ptr1->read(), rgba1, "cloud_1_id",1);

				pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba2(cloud_Data_ptr2->read());
				viewer->removePointCloud("cloud_2_id");
				viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_Data_ptr2->read(), rgba2, "cloud_2_id",2);

				pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba3(cloud_Data_ptr3->read());
				viewer->removePointCloud("cloud_3_id");
				viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_Data_ptr3->read(), rgba3, "cloud_3_id",3);

				pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba4(cloud_Data_ptr4->read());
				viewer->removePointCloud("cloud_4_id");
				viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_Data_ptr4->read(), rgba4, "cloud_4_id",4);

				//cout << viewer->getFPS() << endl;

				viewer->spinOnce();
			}
			if (viewer_excists == 1 && the_interface_scene_visualizer->get_current_4_view_state() == 0)
			{
				viewer->close();
				viewer_excists = 0;
			}
		}
	}

	void scene_visualizer_stop()
	{
		scene_visualizer_state = 0;
		scene_visualizer_thread->join();
	};



};


// this is a simple visuerlizer that makes a single viewport 
// not up to date as i started to use scene_visualizer with the 4 screens instead 
// but included for refrance 
class visualizer
{

public:

	interface* the_interface_visualizer;

	int visualizer_state;

	boost::thread* visualizer_thread;
	int* pointer_signaling_cloud_timing;
	int data_number;

	int viewer_excists;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
	{
		// --------------------------------------------
		// -----Open 3D viewer and add point cloud-----
		// --------------------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);

		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cloud, 0, 255, 0);

		pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);

		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgba, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

		//viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		return viewer;
	}

	void visualizer_start(interface* the_interface, clouddata* cloud_Data_ptr, int* pointer_for_signaling_cloud_vizualizertiming)
	{
		viewer_excists = 0;
		the_interface_visualizer = the_interface;
		pointer_signaling_cloud_timing = pointer_for_signaling_cloud_vizualizertiming;
		visualizer_state = 1;
		data_number = 1;
		visualizer_thread = new boost::thread(boost::bind(&visualizer::visualizer_render, this, cloud_Data_ptr));
	};

	void visualizer_render(clouddata* cloud_Data_ptr)
	{


		while (visualizer_state == 1) // while the thread is not shutting down
		{
			data_number = 1;


			if (viewer_excists == 0 && the_interface_visualizer->get_current_visualizer_state() == 1)
			{
				// window does not excist and the interface wants it setup 

				std::cout << "initierlizing the window setup" << std::endl;

				viewer = customColourVis(cloud_Data_ptr->read());

				viewer_excists = 1;
			}

			//while (*pointer_signaling_cloud_timing == 0)
			//{

			//}


			if (viewer_excists == 1 && the_interface_visualizer->get_current_visualizer_state() == 1)
			{
				//window allready excists just update
				//std::cout << "window excists updating" << std::endl;

				pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud_Data_ptr->read());

				viewer->removePointCloud("sample cloud");
				viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_Data_ptr->read(), rgba, "sample cloud");
				viewer->spinOnce();

			}


			if (viewer_excists == 1 && the_interface_visualizer->get_current_visualizer_state() == 0)
			{
				// interface wants to close window 

				std::cout << "wants to close window closing" << std::endl;

				//vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =  vtkSmartPointer<vtkRenderWindowInteractor>::New();
				viewer->close(); // you might think this works but it does not // known bug in VTK // and yet it works in debug // how ? // you gotta love bugs
				//visualizer->spin(); 
				//visualizer->setSize(1, 1);
				//visualizer_state = 0; 
				//DestroyWindow(static_cast<HWND*>(visualizer->getRenderWindow->GetGenericWindowId())); 



				viewer_excists = 0;

			}



#pragma region
			//if (*pointer_signaling_cloud_timing == 1)
			//{
			//	auto vis_time = std::chrono::high_resolution_clock::now();
			//	// if the 
			//#pragma region



			//#pragma endregion vizualize the cloud by adding udating and removing it 

			//#pragma region
			//	std::stringstream ssx;
			//	ssx << boost::posix_time::microsec_clock::local_time();
			//	std::string s = ssx.str();
			//	std::replace(s.begin(), s.end(), '.', ' ');
			//	std::replace(s.begin(), s.end(), ':', ' ');
			//	std::stringstream ss;
			//	ss << "C:/Users/Julius/ ";
			//	ss << "image data ";
			//	ss << data_number;
			//	ss << " ";
			//	ss << s;
			//	ss << ".bin";
			//#pragma endregion construct the path where to save the cloud data 

			//#pragma region
			//	fstream outfile(ss.str(), ios::binary | ios::out | ios::trunc);
			//	//for (long i = 0; i < cloud_Data_ptr->read()->size(); i++)
			//	//{
			//	//	//std::vector<CustomPoint>& vr = *cloud_Data_ptr->read(); //Create a reference
			//	//	//vr[2].xcord; //Normal access through reference

			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).xcord, sizeof(cloud_Data_ptr->read()->at(i).xcord));
			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).ycord, sizeof(cloud_Data_ptr->read()->at(i).ycord));
			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).zcord, sizeof(cloud_Data_ptr->read()->at(i).zcord));
			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).rcord, sizeof(cloud_Data_ptr->read()->at(i).rcord));
			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).gcord, sizeof(cloud_Data_ptr->read()->at(i).gcord));
			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).bcord, sizeof(cloud_Data_ptr->read()->at(i).bcord));
			//	//	outfile.write((char*)&cloud_Data_ptr->read()->at(i).reflectance, sizeof(cloud_Data_ptr->read()->at(i).reflectance));

			//	//	//cout << cloud_Data_ptr->read()->points[i].x << "  " << cloud_Data_ptr->read()->points[i].y << "  " << cloud_Data_ptr->read()->points[i].z;
			//	//}
			//	outfile.close();
			//	cloud_Data_ptr->clear();
			//	data_number = data_number + 1;
			//#pragma endregion write the data to the file specified in a binary format as this was needed for being able to keep up with 

			//#pragma region
			//	*pointer_signaling_cloud_timing = 0;
			//#pragma endregion signal the networking thread that its done

			//	std::cout << "viewer time " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - vis_time).count() << std::endl;
			//}

#pragma endregion if the mangement thread want the thread to get data 




// this was for saving the point cloud and when it was done saving it it would signal that it was done 
			*pointer_signaling_cloud_timing = 0;



		}
	}

	void visualizer_stop()
	{
		visualizer_state = 0;
		visualizer_thread->join();

		//delete &visualizer;
		//delete stuff
	};

};

















