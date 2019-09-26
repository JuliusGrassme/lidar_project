
#define _CRT_SECURE_NO_DEPRECATE // fopen decrepid warning removal
#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
#define VTK_LEGACY_SILENT

//#pragma once

#define HAVE_BOOST
#pragma warning(disable:4996)

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <chrono>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
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
#include <boost/format.hpp>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
using namespace cv;
using namespace std;
using namespace pcl::tracking;



#include "Classess.h"


/*!
	https://doc.qt.io/qt-5/qdoc-guide-writing.html
	\page generic-guide.html
	\title Generic QDoc Guide
	\nextpage Creating QDoc Configuration Files
	There are three essential materials for generating documentation with qdoc:

	\list
		\li \c qdoc binary
		\li \c qdocconf configuration files
		\li \c Documentation in \c C++, \c QML, and \c .qdoc files
	\endlist
*/
int main(int argc, char* argv[])
{


	int snapshot = 0;
	int* pointer_for_signaling_camera_snapshot = &snapshot;
	int vizualizertiming = 0;
	int* pointer_for_signaling_cloud_vizualizer_timing = &vizualizertiming;


	clouddata  the_clouddata_view_1;
	float image_time = the_clouddata_view_1.load_jpg_image_pointcloud(1468);
	image_time = image_time + 2.15; // 2.1


	clouddata  the_clouddata_view_2;
	the_clouddata_view_2.load_csv_cloud_pointcloud(image_time, false , false ); //load without projection // add image
	cout << "cloud size before " << the_clouddata_view_2.current_frame_pointcloud->points.size() << endl;
	the_clouddata_view_2.subtract_background("C:/testdir/background.csv");
	cout << "cloud size after " << the_clouddata_view_2.current_frame_pointcloud->points.size() << endl;


	clouddata  the_clouddata_view_3;
	the_clouddata_view_3.load_csv_cloud_pointcloud(image_time, true , false); // load with projection // add iamge


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image_cloud     = the_clouddata_view_1.read();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud  = the_clouddata_view_2.read();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projected_cloud = the_clouddata_view_3.read();


	#pragma region this is the part of the code hadeling the segmentation test  http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php#conditional-euclidean-clustering


	clouddata  the_clouddata_view_4;
	//the_clouddata_view_4.load_csv_cloud_pointcloud(image_time, false);

	//cout << " Data containers" << endl;

	// Data containers used
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>), cloud_out(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(*the_clouddata_view_2.current_frame_pointcloud, *cloud_in);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

	//pcl::console::TicToc tt;
	// Load the input point cloud
	// std::cerr << "Loading...\n", tt.tic ();
	// pcl::io::loadPCDFile ("Statues_4.pcd", *cloud_in);
	// std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";
	// Downsample the cloud using a Voxel Grid class
	//std::cerr << "Downsampling...\n", tt.tic();

	//pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
	//vg.setInputCloud(cloud_in);
	//vg.setLeafSize(1.0, 1.0, 1.0);
	//vg.setDownsampleAllData(true);
	//vg.filter(*cloud_out);

	cloud_out = cloud_in;

	//// load the background and subtract it
	//clouddata  the_clouddata_background;
	//the_clouddata_background.load_csv_cloud_pointcloud_single("C:/testdir/background.csv");

	//std::cerr << ">> Done: " << tt.toc() << " ms, " << cloud_out->points.size() << " points\n";
	// Set up a Normal Estimation class and merge data in cloud_with_normals
	//std::cerr << "Computing normals...\n", tt.tic();

	//cout << " Normal Estimation class points " << cloud_out->points.size() << endl;
	pcl::copyPointCloud(*cloud_out, *cloud_with_normals);
	pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
	ne.setInputCloud(cloud_out);
	ne.setSearchMethod(search_tree);
	ne.setRadiusSearch(5);
	ne.compute(*cloud_with_normals);
	//std::cerr << ">> Done: " << tt.toc() << " ms\n";

	//cout << " Conditional Euclidean Clustering class" << endl;
	//std::cerr << "Segmenting to clusters...\n", tt.tic();
	pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec(true);
	cec.setInputCloud(cloud_with_normals);
	cec.setConditionFunction(&customRegionGrowing);
	cec.setClusterTolerance(0.8);
	cec.setMinClusterSize(25);
	cec.setMaxClusterSize(cloud_with_normals->points.size());
	cec.segment(*clusters);
	cec.getRemovedClusters(small_clusters, large_clusters);
	//std::cerr << ">> Done: " << tt.toc() << " ms\n";

	cout << "small clusters size: " << small_clusters->size() << endl;
	// Using the intensity channel for lazy visualization of the output
	for (int i = 0; i < small_clusters->size(); ++i)
	{
		for (int j = 0; j < (*small_clusters)[i].indices.size(); ++j)
		{
			int lr = rand() % 256;
			int lg = rand() % 256;
			int lb = rand() % 256;

			cloud_out->points[(*small_clusters)[i].indices[j]].r = lr;
			cloud_out->points[(*small_clusters)[i].indices[j]].g = 0;
			cloud_out->points[(*small_clusters)[i].indices[j]].b = 0;
		}
	}

	cout << "large clusters size: " << large_clusters->size() << endl;
	for (int i = 0; i < large_clusters->size(); ++i)
	{
		for (int j = 0; j < (*large_clusters)[i].indices.size(); ++j)
		{
			cloud_out->points[(*large_clusters)[i].indices[j]].r = 0;
			cloud_out->points[(*large_clusters)[i].indices[j]].g = 0;
			cloud_out->points[(*large_clusters)[i].indices[j]].b = 255;
		}
	}

	cout << "normal clusters size: " << clusters->size() << endl;
	for (int i = 0; i < clusters->size(); ++i)
	{
		int label = rand() % 256;
		for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
		{
			cloud_out->points[(*clusters)[i].indices[j]].r = 0;
			cloud_out->points[(*clusters)[i].indices[j]].g = label;
			cloud_out->points[(*clusters)[i].indices[j]].b = 0;
		}
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr the_point_cloud_with_all_the_clusters_in(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::copyPointCloud(*cloud_out, *the_point_cloud_with_all_the_clusters_in);

	the_clouddata_view_4.current_frame_pointcloud = the_point_cloud_with_all_the_clusters_in;
	#pragma endregion


	#pragma region extract 1 cluster for test
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr the_cloud_with_the_tracked_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);

	for (int i = 0; i < clusters->size(); ++i)
	{
		int label = rand() % 256;
		for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
		{

			// add points of the 1 cluster to the tracking filter 
			if ( i == 3 )
			{
				pcl::PointXYZRGBA basic_point;
				basic_point.x = cloud_out->points[(*clusters)[i].indices[j]].x;
				basic_point.y = cloud_out->points[(*clusters)[i].indices[j]].y;
				basic_point.z = cloud_out->points[(*clusters)[i].indices[j]].z;
				basic_point.r = cloud_out->points[(*clusters)[i].indices[j]].r;
				basic_point.g = cloud_out->points[(*clusters)[i].indices[j]].g;
				basic_point.b = cloud_out->points[(*clusters)[i].indices[j]].b;
				basic_point.a = cloud_out->points[(*clusters)[i].indices[j]].a;
				the_cloud_with_the_tracked_cluster->points.push_back(basic_point);
			}

		}
	}


	#pragma endregion


	#pragma region setup of the filter 

	// setup of the kulbach liber divergience particle filter
	KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker(8);

	//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker

	

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
	ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY> tracker_;
	tracker_.setTrans(Eigen::Affine3f::Identity());

	tracker_ = tracker;
	
	std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
	default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;
	tracker_.setStepNoiseCovariance(default_step_covariance);

	std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
	tracker_.setInitialNoiseCovariance(initial_noise_covariance);

	std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
	tracker_.setInitialNoiseMean(default_initial_mean);
	tracker_.setIterationNum(1);
	tracker_.setParticleNum(600);
	tracker_.setResampleLikelihoodThr(0.00);
	tracker_.setUseNormal(false);


	//Setup coherence object for tracking
	ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr coherence(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>);

	DistanceCoherence<pcl::PointXYZRGBA>::Ptr distance_coherence(new DistanceCoherence<pcl::PointXYZRGBA>);
	coherence->addPointCoherence(distance_coherence);

	pcl::search::Octree<pcl::PointXYZRGBA>::Ptr search(new pcl::search::Octree<pcl::PointXYZRGBA>(0.01));
	coherence->setSearchMethod(search);
	coherence->setMaximumDistance(0.01);

	tracker_.setCloudCoherence(coherence);




	#pragma endregion


	#pragma region set the input of the filter

	//prepare the model of the target that the tracker is meant to track
	
	// compute the centroid of the object to be tracked
	Eigen::Vector4f c;
	pcl::compute3DCentroid<pcl::PointXYZRGBA>(*the_cloud_with_the_tracked_cluster, c);
	
	// create some afine transform
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
	
	// transform the cloud using that transform
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*the_cloud_with_the_tracked_cluster, *transed_ref, trans.inverse());
	
	// the downsampling, found to increase fps, so thats why its here
	double downsampling_grid_size_ = 0.002;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref_downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
	gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
	
	//set reference model and transformation
	tracker_.setReferenceCloud(transed_ref_downsampled);
	tracker_.setTrans(trans);

	#pragma endregion


	#pragma region run a cloud through the filter set the 3 viewport as its output
	
	tracker_.setInputCloud(the_point_cloud_with_all_the_clusters_in);
	tracker_.compute();

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr particles( new pcl::PointCloud< pcl::PointXYZRGBA > );

	auto temp_particales = tracker_.getParticles();

	for (int s = 0; s < temp_particales->points.size(); s++)
	{
		pcl::PointXYZRGBA basic_point;
		basic_point.x = temp_particales->points[s].x;
		basic_point.y = temp_particales->points[s].y;
		basic_point.z = temp_particales->points[s].z;
		basic_point.r = 0;
		basic_point.g = 128;
		basic_point.b = 255;
		basic_point.a = 255;
		particles->points.push_back(basic_point);
	}

	the_clouddata_view_3.current_frame_pointcloud = particles;

	#pragma endregion


	#pragma region this region describes the section of the code used to make the 4 viewport display do the dataprocessing and make the interface for it
	scene_visualizer the_scene_visualizer;
	interface  the_interface;

	the_scene_visualizer.scene_visualizer_start(&the_interface, &the_clouddata_view_1, &the_clouddata_view_2, &the_clouddata_view_3, &the_clouddata_view_4, pointer_for_signaling_cloud_vizualizer_timing);
	the_interface.interface_start();
	#pragma endregion


	#pragma region this region describes the section of the code used to interface directly with the lidar and camera, when using this the  4 viewport display section should be uncomented

	//clouddata  the_clouddata; // this is the data class for holding a pointcloud
	//networking the_networking;
	//visualizer the_visualizer;
	//video      the_video;

	//the_networking.networking_start(&the_interface, &the_clouddata, pointer_for_signaling_camera_snapshot, pointer_for_signaling_cloud_vizualizer_timing);
	//the_visualizer.visualizer_start(&the_interface, &the_clouddata, pointer_for_signaling_cloud_vizualizer_timing);
	//the_video.video_start(&the_interface, pointer_for_signaling_camera_snapshot);
	//the_scene_visualizer.scene_visualizer_start( &the_interface, &the_clouddata_view_1, &the_clouddata_view_2, &the_clouddata_view_3, &the_clouddata_view_4 );
	
	//the_visualizer.visualizer_stop();
	//the_networking.networking_stop();

	#pragma endregion

}







