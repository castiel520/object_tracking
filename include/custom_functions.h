#ifndef CUSTOM_FUNCTIONS_H
#define CUSTOM_FUNCTIONS_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/integral_image_normal.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


const float bad_point = std::numeric_limits<float>::quiet_NaN();


	// Transforms "cloud_in" in "cloud_out" using "transform" pose
//~ template <typename PointT> 
void transformPointCloudCustom(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out, const Eigen::Affine3d &transform);
void transformPointCloudCustom(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, const Eigen::Affine3d &transform);
float getAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB);
float getAngle(Eigen::Vector4f vectorA, Eigen::Vector4f vectorB);
float getNegativeAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB);
float getNegativeHorizontalAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB);
float getHorizontalAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB);
int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud);
int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius);
pcl::PointXYZ getPointXYZ(Eigen::Vector3f vector);
Eigen::Affine3d createAffine3d(Eigen::Matrix3d rotation, Eigen::Vector3d translation);
Eigen::Affine3d createAffine3d(Eigen::Matrix3f rotation, Eigen::Vector3f translation);
Eigen::Affine3f createAffine3f(Eigen::Matrix3f rotation, Eigen::Vector3f translation);
void sayAffine(Eigen::Affine3d matrix);
void sayAffine(Eigen::Affine3f matrix);
Eigen::Vector3f transformPoint (Eigen::Vector3f point, Eigen::Affine3f T);
std::vector<Eigen::Vector3f> transformPoint (std::vector<Eigen::Vector3f> point_in, Eigen::Affine3f T);
cv::Mat falseColor(cv::Mat img_depth);
Eigen::Matrix3d quatToMatrix(Eigen::Vector4d q);
void imshow(std::string name, cv::Mat image);
cv::Mat canny(cv::Mat img, float th1, float th2);
cv::Mat edges_from_normals(cv::Mat img, cv::Mat depth, float thresh);
cv::Mat canny_three_channels(cv::Mat img, float th1, float th2);
cv::Mat sobel(cv::Mat img);
void thinningIteration(cv::Mat& im, int iter);
void thinning(cv::Mat& im);
Eigen::Vector4f getPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius);
cv::Mat normalim(cv::Mat depth);
pcl::PointCloud<pcl::PointXYZ>::Ptr compute3Dpoints(cv::Mat depth, cv::Mat K);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute3Dpoints(cv::Mat depth, cv::Mat K,cv::Mat color, cv::Mat K_color,cv::Mat R, cv::Mat T);
pcl::PointCloud<pcl::PointXYZ>::Ptr compute3DpointsOrganized(cv::Mat depth, cv::Mat K);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute3DpointsOrganized(cv::Mat depth, cv::Mat K,cv::Mat color, cv::Mat K_color,cv::Mat R, cv::Mat T);

void projectionToOpenGLRBBottomLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar);
void projectionToOpenGLTopLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar);
void projectionToOpenGLBottomLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar);
void projectionToOpenGL( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar);
void detectionToOpenGL(const Eigen::Matrix4f &M, double m[16], const cv::Mat &K, int imageWidth, int imageHeight,
      float near, float far);



#endif
