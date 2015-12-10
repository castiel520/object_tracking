#ifndef BACKGROUND
#define BACKGROUND

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/registration/icp.h>


#include "custom_functions.h"

class BackGround {

public:    
    BackGround();
    BackGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    ~BackGround();

    void findTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals);
    void computeCamera2TableMatrix ();
    
//private:

    Eigen::VectorXf plane_vector;
    pcl::ModelCoefficients::Ptr model_coefficients;
    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud_concavehull;

    float height,length,width;
    Eigen::Affine3f t2c, c2t;

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertex2t, vertex2c;
};
#endif
