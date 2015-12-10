#ifndef PC_CROPPER
#define PC_CROPPER 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>

class PCCropper {

public:
    
    PCCropper();
    PCCropper(const cv::Mat &K, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &T);
    ~PCCropper();

    void initialize(const cv::Mat &K, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &T);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop(const cv::Mat &color, const cv::Mat &depth,
                                                const cv::Point2f& p1,const cv::Point2f& p2 );

private:
    //calibration matrices
    cv::Mat K_depth_,K_color_;
    //transformation between cameras
    cv::Mat R_,t_;
   

   

};
#endif