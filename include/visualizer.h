#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vtkTransform.h>

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

//#include "plane.h"
//#include "stair_classes.h"

// CLASS VISUALIZER
struct Viewer
{
	// Constructor with standard settings
	Viewer() : cloud_viewer_ ("Viewer")
	{
        cloud_viewer_.setBackgroundColor (0.5, 0.5, 0.5); // black
//		cloud_viewer_.setBackgroundColor (255, 255, 255); // white
//        cloud_viewer_.addCoordinateSystem (0.05);
        cloud_viewer_.setPosition (0, 0);


		// // Values from
		// 0.718805,8.73687/				camera clip
		// -0.0751458,0.206077,1.13894/		camera pos
		// 0.0238492,1.90784,-2.41265/		camera focal
		// 0.00252639,0.901789,0.432168/	camera view
		// 0.523599/						fov
		// 640,480/							window size
		// 66,52 							window pos


        // cloud_viewer_.setSize (640, 480);
        // cloud_viewer_.setCameraClipDistances (0.01, 10.01);
        // // cloud_viewer_.setCameraClipDistances(0.75,9);
        // cloud_viewer_.setCameraPosition(	0,		1.90,	-2.5, 	// focal
        // 									-0.075,	0.2,	1.2,  	// pos
        // 									0.0025,	0.9,	0.45);	// view
        // cloud_viewer_.setCameraFieldOfView(0.57);



        // 14 - 10 - 2015
        // 0.00205575,2.05575/
        // 0.0192092,0.23056,-0.206419/
        // -0.0356287,-0.393573,0.332528/
        // -0.0851805,0.655469,0.750403/
        // 1/
        // 640,480/115,74


//       cloud_viewer_.setCameraClipDistances (0.00205575, 2.05575);
//       cloud_viewer_.setCameraPosition(	 -0.0356287,-0.393573,0.332528  ,
//                                         0.0192092,0.23056,-0.206419, // pos
//                                           -0.0851805,0.655469,0.750403 // focal
//                                           );	// view
//       cloud_viewer_.setCameraFieldOfView(1);
//       cloud_viewer_.setSize (640, 480);


        // Default conf
			
        cloud_viewer_.setSize (640, 480);
        cloud_viewer_.setCameraClipDistances (0.0, 10.00);
        // cloud_viewer_.setCameraClipDistances(0.75,9);
        cloud_viewer_.setCameraPosition(	0,	0,	0,  	// pos
                                            0,	0,	1, // focal
                                            0,	-1,	0);	// view
        cloud_viewer_.setCameraFieldOfView(1);




		name_ = "camera";    
		
		// cloud_viewer_.addText ("No ascending stairs", 50, 50, 20, 1.0f, 1.0f, 1.0f, "uptext");
		// cloud_viewer_.addText ("No descending stairs", 50, 100, 20, 1.0f, 1.0f, 1.0f, "downtext");
		// cloud_viewer_.addText ("On the ground", 50, 150, 20, 1.0f, 1.0f, 1.0f, "infotext");
		
		iteration = 0;
	}

	// Adds a cloud to the viewer
	void drawCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double r, double g, double b, int index);
	void drawColorCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int index);
	void drawSphere (pcl::PointXYZ point, double radius, double r, double g, double b, int index);
	void drawNormals (pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
    void drawRectangle (pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, double r, double g, double b, double opacity, std::string string);

	// DRAWBOX
	/* z_dir, y_dir, x_dir are the lenghts in the direction of the colums of the dir matrix (3,2,1).
	 * For the stair_dir, we chose the x_dir to be in the length, but it changes with other dir matrices.
	 * 
	 */
	void drawBox(pcl::PointXYZ centroid, float z_dir, float y_dir, float x_dir, Eigen::Matrix3f dir, int Q);

	// Create the axis_points which will be moved in the viewer with the updated pose
	void createAxis ();
	// Rotates the axis_points and draws them in the viewer
	void drawAxis (Eigen::Affine3d& pose);
	void drawAxis (Eigen::Matrix3f pose);


    void snapPLYmodel(const std::string& plyModel, Eigen::Matrix4d T, const std::string &name);

	Eigen::Affine3f viewer_pose_;
	std::string name_;
	pcl::visualization::PCLVisualizer cloud_viewer_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points;
	int iteration;
	public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
