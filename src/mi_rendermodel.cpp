#include <stdio.h>
#include <iomanip>
#include <string>
#include <iostream>
#include <limits>

#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


#include <obj_recognition/image_cropper.h>
#include <obj_recognition/obj_recognition.h>
#include <object_recognition_renderer/renderer3d.h>

//PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include "visualizer.h"
#include "custom_functions.h"

#include <GL/glut.h>

//const float bad_point = std::numeric_limits<float>::quiet_NaN();



//std::vector<cv::Point3d> boxModel()
//{
//    std::vector<cv::Point3d> points;
//    cv::Point3d p;
//    p = cv::Point3f(0.024,-0.085,0);
//    points.push_back(p);
//    p = cv::Point3f(-0.024,-0.085,0);
//    points.push_back(p);
//    p = cv::Point3f(-0.024,-0.085,0.094);
//    points.push_back(p);
//    p = cv::Point3f(0.024,-0.085,0.094);
//    points.push_back(p);
//    p = cv::Point3f(0.024,0.085,0);
//    points.push_back(p);
//    p = cv::Point3f(-0.024,0.085,0);
//    points.push_back(p);

//    return points;
//}

std::vector<cv::Point3d> boxModel()
{
    std::vector<cv::Point3d> points;
    cv::Point3d p;
    p = cv::Point3f(0.085,-0.025,0.0485);
    points.push_back(p);
    p = cv::Point3f(0.085,0.025,0.0485);
    points.push_back(p);
    p = cv::Point3f(0.085,0.025,-0.0465);
    points.push_back(p);
    p = cv::Point3f(0.085,-0.025,-0.0465);
    points.push_back(p);
    p = cv::Point3f(-0.085,-0.025,0.0485);
    points.push_back(p);
    p = cv::Point3f(-0.085,0.025,0.0485);
    points.push_back(p);

    return points;
}

//Eigen::Vector4f getPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//{
//    // Create the segmentation object for the planar model and set all the parameters
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (1000);
//    seg.setDistanceThreshold (0.01);//0.04

//    // Segment the largest planar component from the remaining cloud
//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);


//    Eigen::Vector4f plane_vector(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);

//    double angle = pcl::getAngle3D(plane_vector,Eigen::Vector4f(0,2,2,1));
//    if (pcl::rad2deg(angle) < 90)
//        plane_vector = -plane_vector;

//    pcl::ExtractIndices<pcl::PointXYZ> ex;
//    ex.setInputCloud (cloud);
//    ex.setIndices (inliers);
//    ex.setNegative (false);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    ex.filter (*temp_cloud);

//    cloud.reset(new pcl::PointCloud<pcl::PointXYZ> );
//    *cloud = *temp_cloud;

//    return plane_vector;
//}

Eigen::Vector4f getPlaneWithNormals (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,  pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
    pcl::SampleConsensusModelNormalPlane<pcl::PointXYZ, pcl::Normal>::Ptr model(new pcl::SampleConsensusModelNormalPlane<pcl::PointXYZ,pcl::Normal> (cloud_in));
    model->setInputNormals(normals);
    model->setNormalDistanceWeight(0.2);
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model);
    sac.setMaxIterations(100);
    sac.setDistanceThreshold(0.01);

    bool result = sac.computeModel ();

    std::vector<int> inliers;
    Eigen::VectorXf coefficients;

    sac.getModelCoefficients(coefficients);
    sac.getInliers (inliers);

    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    indices->indices = inliers;

    Eigen::Vector4f plane_vector;//(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
    plane_vector = coefficients;


    double angle = pcl::getAngle3D(plane_vector,Eigen::Vector4f(0,2,2,1));
    if (pcl::rad2deg(angle) < 90)
        plane_vector = -plane_vector;

    //    pcl::ExtractIndices<pcl::PointXYZ> ex;
    //    ex.setInputCloud (cloud_in);

    //    ex.setIndices (indices);
    //    ex.setNegative (false);
    //    ex.filter (*temp_cloud_in);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (indices);
    proj.setInputCloud (cloud_in);
    pcl::ModelCoefficients::Ptr model_coeffs (new pcl::ModelCoefficients);
    for (int i = 0; i<4; i++)
        model_coeffs->values.push_back(coefficients[i]);
    proj.setModelCoefficients (model_coeffs);
    proj.filter (*temp_cloud);

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (temp_cloud);
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);

    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ> );
    *cloud_in = *temp_cloud;
    cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ> );
    *cloud_out = *cloud_hull;

    return plane_vector;
}

//cv::Mat normalim(cv::Mat depth)
//{
//    cv::Mat depth_filtered;
//    cv::bilateralFilter(depth,depth_filtered, 5, 10, 10, cv::BORDER_DEFAULT );
////    cv::GaussianBlur(depth, depth_filtered, cv::Size(3,3), 1, 1 );
////    cv::medianBlur(depth,depth_filtered,5);
////    depth_filtered = depth;
//    cv::Mat normal(depth.rows,depth.cols,CV_8UC3);

//    cv::Mat kernelH(3, 3, CV_32F, cv::Scalar(0));
//    kernelH.at<float>(1,0) = 0.5f;
//    kernelH.at<float>(1,2) = -0.5f;

//    cv::Mat kernelV(3, 3, CV_32F, cv::Scalar(0));
//    kernelV.at<float>(0,1) = -0.5f;
//    kernelV.at<float>(2,1) = 0.5f;

////    cv::Mat kernelH(5, 5, CV_32F, cv::Scalar(0));
////    kernelH.at<float>(2,0) = 0.5f;
////    kernelH.at<float>(2,1) = 1.0f;
////    kernelH.at<float>(2,3) = -1.0f;
////    kernelH.at<float>(2,4) = -0.5f;
////    kernelH /= 3;

////    cv::Mat kernelV(5, 5, CV_32F, cv::Scalar(0));
////    kernelV.at<float>(0,2) = -0.5f;
////    kernelV.at<float>(1,2) = -1.0f;
////    kernelV.at<float>(3,2) = 1.0f;
////    kernelV.at<float>(4,2) = 0.5f;
////    kernelV /= 3;

////    cv::Mat kernelH(3, 3, CV_32F, cv::Scalar(0));
////    kernelH.at<float>(0,0) = -1.0f;
////    kernelH.at<float>(1,0) = -2.0f;
////    kernelH.at<float>(2,0) = -1.0f;
////    kernelH.at<float>(0,2) = 1.0f;
////    kernelH.at<float>(1,2) = 2.0f;
////    kernelH.at<float>(2,2) = 1.0f;

////    cv::Mat kernelV(3, 3, CV_32F, cv::Scalar(0));
////    cv::transpose(kernelH,kernelV);


//    cv::Mat gradientx, gradienty;
//    cv::filter2D(depth_filtered, gradientx,-1,kernelH,cv::Point(-1,-1),0,cv::BORDER_DEFAULT );
//    cv::filter2D(depth_filtered, gradienty,-1,kernelV,cv::Point(-1,-1),0,cv::BORDER_DEFAULT );

//    //compute the magnitude
//    cv::Mat magnitude =  gradientx.mul(gradientx) +  gradienty.mul(gradienty);
//    magnitude+=1.0;
//    cv::sqrt(magnitude,magnitude);

//    for (int i=0;i< magnitude.rows; i++)
//        for(int j =0; j < magnitude.cols;j++){
//            cv::Vec3b *cn = &normal.at<cv::Vec3b>(i,j );
//            if (depth.at<float>(i,j) > 0)
//            {
//                float nx,ny,nz;

//                nx = gradientx.at<float>(i,j)/magnitude.at<float>(i,j);
//                ny  = -gradienty.at<float>(i,j)/magnitude.at<float>(i,j);
//                nz = 1.0/magnitude.at<float>(i,j);
//                cn->val[2] = (uchar) ((nx + 1.0) /2.0 * 255.0);
//                cn->val[1] = (uchar) ((ny + 1.0) /2.0 * 255.0);
//                cn->val[0] = (uchar) ((nz + 1.0) /2.0 * 255.0);
////                cn->val[2] = nx;
////                cn->val[1] = ny;
////                cn->val[0] = nz;
//            }
//            else
//            {
//                cn->val[2] = 0;
//                cn->val[1] = 0;
//                cn->val[0] = 0;
//            }

//        }

//    return normal;
//}

cv::Mat computeNormalMap(cv::Mat depth, cv::Mat K)
{
//    cv::Mat mat_xyz(depth.rows,depth.cols,CV_32FC3);
//    mat_xyz = cv::Scalar(0,0,0);
    cv::Mat mat_normals(depth.rows,depth.cols,CV_32FC3);
    mat_normals = cv::Scalar(0,0,0);

    Eigen::MatrixXf mat_x = Eigen::MatrixXf::Zero(depth.rows,depth.cols);
    Eigen::MatrixXf mat_y = Eigen::MatrixXf::Zero(depth.rows,depth.cols);
    Eigen::MatrixXf mat_z = Eigen::MatrixXf::Zero(depth.rows,depth.cols);

    Eigen::MatrixXf n_x = Eigen::MatrixXf::Zero(depth.rows,depth.cols);
    Eigen::MatrixXf n_y = Eigen::MatrixXf::Zero(depth.rows,depth.cols);
    Eigen::MatrixXf n_z = Eigen::MatrixXf::Zero(depth.rows,depth.cols);

    cv::Point3d point;

    cv::Mat res;
    cv::Mat Kinv = K.inv();

//    pcl::PointXYZ p;

    for (std::size_t i=0; i < depth.rows; i++)
    {
        for (std::size_t j=0; j < depth.cols; j++)
        {
            point.x = j;
            point.y = i;
            point.z = 1;

            res = Kinv * cv::Mat( point, false);
            res.copyTo(cv::Mat ( point, false));



            if (depth.at<float>(i,j)>0.0)
            {
                point *= depth.at<float>(i,j)/1000.;

                mat_x(i,j) = point.x;
                mat_y(i,j) = point.y;
                mat_z(i,j) = point.z;

                std::cout << depth.at<float>(i,j) << std::endl << std::endl;
                std::cout << point.x << " " << point.y << " " << point.z << std::endl << std::endl;
            }
            if ((i > 0) && (i < depth.rows - 1) && (j > 0) && (j < depth.cols - 1))
            {
                if ((depth.at<float>(i-1,j-1) > 0.0) && (depth.at<float>(i-1,j) > 0.0) && (depth.at<float>(i,j-1) > 0.0))
                {
                    Eigen::Vector3f v1, v2;
                    v1 = Eigen::Vector3f(mat_x(i,j-1) - mat_x(i-1,j-1), mat_y(i,j-1) - mat_y(i-1,j-1), mat_z(i,j-1) - mat_z(i-1,j-1));
                    v2 = Eigen::Vector3f(mat_x(i-1,j) - mat_x(i-1,j-1), mat_y(i-1,j) - mat_y(i-1,j-1), mat_z(i-1,j) - mat_z(i-1,j-1));
                    Eigen::Vector3f normal;
                    v1.normalize();
                    v2.normalize();
                    normal = v1.cross(v2);
                    normal.normalize();

                    std::cout << normal << std::endl << std::endl;
                    std::cout << v1 << std::endl;
                    std::cout << v2 << std::endl << std::endl;

                    n_x(i-1,j-1) = (normal[0]+1)/2*255;
                    n_y(i-1,j-1) = (normal[1]+1)/2*255;
                    n_z(i-1,j-1) = (normal[2]+1)/2*255;
//                    n_x(i-1,j-1) = (normal[0])*255;
//                    n_y(i-1,j-1) = (normal[1])*255;
//                    n_z(i-1,j-1) = (normal[2])*255;
                }
            }
        }
    }

    cv::Mat normals_x, normals_y, normals_z;
    cv::eigen2cv(n_x,normals_x);
    cv::eigen2cv(n_y,normals_y);
    cv::eigen2cv(n_z,normals_z);

    std::vector<cv::Mat> array_to_merge;

//    imshow("normals_x",normals_x);
//    imshow("normals_y",normals_y);
//    imshow("normals_z",normals_z);

    array_to_merge.push_back(normals_x);
    array_to_merge.push_back(normals_y);
    array_to_merge.push_back(normals_z);

    cv::Mat color;

    cv::merge(array_to_merge, color);


    mat_normals.convertTo(mat_normals,CV_8UC3);
//    imshow("normals",color);
//    cv::waitKey();
//    return pc;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr compute3Dpoints(cv::Mat depth, cv::Mat K)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

//    cv::Point3d point;

//    cv::Mat res;
//    cv::Mat Kinv = K.inv();

//    pcl::PointXYZ p;

//    for (std::size_t i=0; i < depth.rows; i++)
//    {
//        for (std::size_t j=0; j < depth.cols; j++)
//        {
//            point.x = j;
//            point.y = i;
//            point.z = 1;

//            res = Kinv * cv::Mat( point, false);
//            res.copyTo(cv::Mat ( point, false));

//            if (depth.at<float>(i,j)>0.0)
//            {
//                point *= depth.at<float>(i,j)/1000.;

//                p.x = point.x;
//                p.y = point.y;
//                p.z = point.z;

//                pc->push_back(p);
//            }
//        }
//    }
//    return pc;
//}

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute3Dpoints(cv::Mat depth, cv::Mat K,cv::Mat color, cv::Mat K_color,cv::Mat R, cv::Mat T)
//{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>);
//    cv::Point3d point;

//    cv::Mat res;
//    cv::Mat Kinv = K.inv();
//    cv::Mat Rtrans;
//    cv::transpose(R,Rtrans);

//    pcl::PointXYZRGB pcolor;

//    for (std::size_t i=0; i < depth.rows; i++)
//    {
//        for (std::size_t j=0; j < depth.cols; j++)
//        {

//            int u,v;
//            point.x = j;
//            point.y = i;
//            point.z = 1;

//            res = Kinv * cv::Mat( point, false);
//            res.copyTo(cv::Mat ( point, false));

//            if (depth.at<float>(i,j)>0.0)
//            {
//                point *= depth.at<float>(i,j)/1000.;

//                res = R *  cv::Mat( point, false) +  T ;
//                res.copyTo(cv::Mat ( point, false));

//                pcolor.x = point.x;
//                pcolor.y = point.y;
//                pcolor.z = point.z;

//                res = K_color * cv::Mat( point,false);
//                res.copyTo(cv::Mat ( point, false));

//                v = (int) (point.x / point.z + .5);
//                u = (int) (point.y / point.z + .5);

//                if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
//                {
//                    uchar pr, pg, pb;

//                    //Get RGB info
//                    pb = color.at<cv::Vec3b>(u,v)[0];
//                    pg = color.at<cv::Vec3b>(u,v)[1];
//                    pr = color.at<cv::Vec3b>(u,v)[2];


//                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
//                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);



//                }
//                else
//                {
////                    pcolor.r = 0;
////                    pcolor.g = 255;
////                    pcolor.b = 0;
//                    pcolor.r = 255;
//                    pcolor.g = 255;
//                    pcolor.b = 255;
//                }
//                pcColor->push_back(pcolor);
//            }
//        }
//    }
//    return pcColor;
//}
//pcl::PointCloud<pcl::PointXYZ>::Ptr compute3DpointsOrganized(cv::Mat depth, cv::Mat K)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

//    cv::Point3d point;

//    cv::Mat res;
//    cv::Mat Kinv = K.inv();

//    pcl::PointXYZ p;

//    for (std::size_t i=0; i < depth.rows; i++)
//    {
//        for (std::size_t j=0; j < depth.cols; j++)
//        {

//            int u,v;
//            point.x = j;
//            point.y = i;
//            point.z = 1;

//            res = Kinv * cv::Mat( point, false);
//            res.copyTo(cv::Mat ( point, false));

//            if (depth.at<float>(i,j)>0.0)
//            {
//                point *= depth.at<float>(i,j)/1000.;

//                p.x = point.x;
//                p.y = point.y;
//                p.z = point.z;

//            }
//            else
//            {
//                p.x = bad_point;
//                p.y = bad_point;
//                p.z = bad_point;
//            }
//            pc->push_back(p);
//        }
//    }
//    pc->height = 480;
//    pc->width = 640;
//    pc->is_dense = false;
//    return pc;
//}

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute3DpointsOrganized(cv::Mat depth, cv::Mat K,cv::Mat color, cv::Mat K_color,cv::Mat R, cv::Mat T)
//{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>);

//    cv::Point3d point;

//    cv::Mat res;
//    cv::Mat Kinv = K.inv();
////    cv::Mat Rtrans;
////    cv::transpose(R,Rtrans);

//    pcl::PointXYZRGB pcolor;

//    for (std::size_t i=0; i < depth.rows; i++)
//    {
//        for (std::size_t j=0; j < depth.cols; j++)
//        {

//            int u,v;
//            point.x = j;
//            point.y = i;
//            point.z = 1;

//            res = Kinv * cv::Mat( point, false);
//            res.copyTo(cv::Mat ( point, false));

//            if (depth.at<float>(i,j)>0.0)
//            {
//                point *= depth.at<float>(i,j)/1000.;

//                res = R *  cv::Mat( point, false) +  T ;
//                res.copyTo(cv::Mat ( point, false));

//                pcolor.x = point.x;
//                pcolor.y = point.y;
//                pcolor.z = point.z;

//                res = K_color * cv::Mat( point,false);
//                res.copyTo(cv::Mat ( point, false));

//                v = (int) (point.x / point.z + .5);
//                u = (int) (point.y / point.z + .5);

//                if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
//                {
//                    uchar pr, pg, pb;

//                    //Get RGB info
//                    pb = color.at<cv::Vec3b>(u,v)[0];
//                    pg = color.at<cv::Vec3b>(u,v)[1];
//                    pr = color.at<cv::Vec3b>(u,v)[2];


//                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
//                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);

//                }
//                else
//                {
////                    pcolor.x = bad_point;
////                    pcolor.y = bad_point;
////                    pcolor.z = bad_point;
////                    pcolor.rgb = bad_point;
//                    pcolor.r = 0;
//                    pcolor.g = 255;
//                    pcolor.b = 0;
//                }
//            }
//            else
//            {
//                pcolor.x = bad_point;
//                pcolor.y = bad_point;
//                pcolor.z = bad_point;
//                pcolor.r = 255;
//                pcolor.g = 0;
//                pcolor.b = 0;
//            }

//            pcColor->push_back(pcolor);
//        }
//    }
//    pcColor->height = 480;
//    pcColor->width = 640;
//    pcColor->is_dense = false;
//    return pcColor;
//}

////pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
////{
////    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

////    if (cloud->width == 640)
////    {
////        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
////        normal_estimator.setNormalEstimationMethod (normal_estimator.AVERAGE_3D_GRADIENT);
////        normal_estimator.setMaxDepthChangeFactor(0.02f);
////        normal_estimator.setNormalSmoothingSize(10.f);
////        normal_estimator.setInputCloud(cloud);
////        normal_estimator.compute(*normals);
////    }
////    else
////    {
////        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
////        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
////        normal_estimator.setSearchMethod (tree);
////        normal_estimator.setInputCloud (cloud);
////        if (radius < 1)
////            normal_estimator.setRadiusSearch (radius);
////        else
////            normal_estimator.setKSearch( (int) radius);
////        normal_estimator.compute (*normals);
////    }

////    return normals;
////}

Eigen::Affine3f computeCamera2TableMatrix (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr contour, pcl::PointCloud<pcl::PointXYZ>::Ptr & points)
{
    Eigen::Matrix3f covariance;
    Eigen::Vector4f vector_centroid;
    pcl::compute3DCentroid(*cloud,vector_centroid);
    pcl::computeCovarianceMatrixNormalized(*cloud,vector_centroid,covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    Eigen::Matrix3f eigDx_aux = eigDx;
    eigDx_aux.col(0) = eigDx.col(2);
    eigDx_aux.col(2) = eigDx.col(0);
    eigDx = eigDx_aux;

    if (getAngle(eigDx.col(0),Eigen::Vector3f(1,0,0)) > 90*M_PI/180)
        eigDx.col(0) = -eigDx.col(0);

    if (getAngle(eigDx.col(1),Eigen::Vector3f(0,-1,1)) > 90*M_PI/180)
        eigDx.col(1) = -eigDx.col(1);

    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    float height = max_pt.z - min_pt.z;
    float length = max_pt.y - min_pt.y;
    float width = max_pt.x - min_pt.x;
    float initial_area = width*length;

    // Optimize rectangle
    Eigen::Matrix3f rot(Eigen::Matrix3f::Identity());
    float angle = M_PI/180;
    rot.row(0)  = Eigen::Vector3f(cos(angle), sin(angle),0);
    rot.row(1)  = Eigen::Vector3f(-sin(angle), cos(angle),0);
    Eigen::Matrix3f  dir_rot = eigDx*rot;

    p2w.block<3,3>(0,0) = dir_rot.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
    pcl::transformPointCloud(*contour, cPoints, p2w);
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    float current_area = (max_pt.x - min_pt.x)*(max_pt.y - min_pt.y);

    if (current_area < initial_area)
    {
        float count = 2;
        Eigen::Matrix3f prev_eigDx = dir_rot;
        while(current_area < initial_area)
        {
            initial_area = current_area;
            prev_eigDx = dir_rot;

            angle = count*M_PI/180;
            rot.row(0)  = Eigen::Vector3f(cos(angle), sin(angle),0);
            rot.row(1)  = Eigen::Vector3f(-sin(angle), cos(angle),0);
            dir_rot = eigDx*rot;

            p2w.block<3,3>(0,0) = dir_rot.transpose();
            p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
            pcl::transformPointCloud(*contour, cPoints, p2w);
            pcl::getMinMax3D(cPoints, min_pt, max_pt);
            current_area = (max_pt.x - min_pt.x)*(max_pt.y - min_pt.y);

            count = count+1;
        }
        eigDx = prev_eigDx;
    }
    else
    {
        float count = 1;
        Eigen::Matrix3f prev_eigDx = eigDx;
        current_area = 0;
        while(current_area < initial_area)
        {
            if (count > 1)
                initial_area = current_area;
            prev_eigDx = dir_rot;

            angle = -count*M_PI/180;
            rot.row(0)  = Eigen::Vector3f(cos(angle), sin(angle),0);
            rot.row(1)  = Eigen::Vector3f(-sin(angle), cos(angle),0);
            dir_rot = eigDx*rot;

            p2w.block<3,3>(0,0) = dir_rot.transpose();
            p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
            pcl::transformPointCloud(*contour, cPoints, p2w);
            pcl::getMinMax3D(cPoints, min_pt, max_pt);
            current_area = (max_pt.x - min_pt.x)*(max_pt.y - min_pt.y);

            count = count+1;
        }
        eigDx = prev_eigDx;
    }

    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
    pcl::transformPointCloud(*contour, cPoints, p2w);
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    Eigen::Affine3f t2c = createAffine3f(eigDx,vector_centroid.head<3>());

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    height = max_pt.z - min_pt.z;
    length = max_pt.y - min_pt.y;
    width = max_pt.x - min_pt.x;
    Eigen::Vector3f p1 = Eigen::Vector3f(-width/2, +length/2,0);
    Eigen::Vector3f p2 = Eigen::Vector3f(+width/2, +length/2,0);
    Eigen::Vector3f p3 = Eigen::Vector3f(+width/2, -length/2,0);
    Eigen::Vector3f p4 = Eigen::Vector3f(-width/2, -length/2,0);

    points.reset(new  pcl::PointCloud<pcl::PointXYZ>);
    points->push_back(getPointXYZ(p1));
    points->push_back(getPointXYZ(p2));
    points->push_back(getPointXYZ(p3));
    points->push_back(getPointXYZ(p4));
    points->push_back(pcl::PointXYZ(0,0,0));

    mean_diag = transformPoint(mean_diag,t2c);

    t2c = createAffine3f(eigDx,mean_diag);
    Eigen::Affine3f c2t = t2c.inverse();

    return c2t;
}


//void projectionToOpenGLRBBottomLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
//{
//    float fu = K.at<double>(0,0);
//    float fv = (float)K.at<double>(1, 1);
//    float u0 = (float)K.at<double>(0, 2);
//    float v0 = (float)K.at<double>(1, 2);
    
//    float L = +(u0) * zNear / -fu;
//    float T = +(v0) * zNear / fv;
//    float R = -(w-u0) * zNear / -fu;
//    float B = -(h-v0) * zNear / fv;
    
//    std::fill_n(p,4*4,0);
    
//    p[0*4+0] = 2 * zNear / (R-L);
//    p[1*4+1] = 2 * zNear / (T-B);
//    p[2*4+2] = -(zFar +zNear) / (zFar - zNear);
//    p[2*4+0] = (R+L)/(R-L);
//    p[2*4+1] = (T+B)/(T-B);
//    p[2*4+3] = -1.0;
//    p[3*4+2] =  -(2*zFar*zNear)/(zFar-zNear);
    
//}

//void projectionToOpenGLTopLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
//{
    
//    float fu = K.at<double>(0,0);
//    float fv = (float)K.at<double>(1, 1);
//    float u0 = (float)K.at<double>(0, 2);
//    float v0 = (float)K.at<double>(1, 2);

//    float L = -(u0) * zNear / fu;
//    float R = +(w-u0) * zNear / fu;
//    float T = -(v0) * zNear / fv;
//    float B = +(h-v0) * zNear / fv;
        
//    std::fill_n(p,4*4,0);
    
//    p[0*4+0] = 2 * zNear / (R-L);
//    p[1*4+1] = 2 * zNear / (T-B);
//    p[2*4+0] = (R+L)/(L-R);
//    p[2*4+1] = (T+B)/(B-T);
//    p[2*4+2] = (zFar +zNear) / (zFar - zNear);
//    p[2*4+3] = 1.0;
//    p[3*4+2] =  (2*zFar*zNear)/(zNear - zFar);

//}

//void projectionToOpenGLBottomLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
//{
 
//     float fu = K.at<double>(0,0);
//    float fv = (float)K.at<double>(1, 1);
//    float u0 = (float)K.at<double>(0, 2);
//    float v0 = (float)K.at<double>(1, 2);
     
//    float L = -(u0) * zNear / fu;
//    float T = +(h-v0) * zNear / fv;
//    float R = +(w-u0) * zNear / fu;
//    float B = -(v0) * zNear / fv;
    
//     std::fill_n(p,4*4,0);
    
//    p[0*4+0] = 2 * zNear / (R-L);
//    p[1*4+1] = 2 * zNear / (T-B);
//    p[2*4+0] = (R+L)/(L-R);
//    p[2*4+1] = (T+B)/(B-T);
//    p[2*4+2] = (zFar +zNear) / (zFar - zNear);
//    p[2*4+3] = 1.0;
//    p[3*4+2] =  (2*zFar*zNear)/(zNear - zFar);
    
    
//    Eigen::Matrix4d M(p);
    
//    std::cout << "Matrix BL: \n" << M << std::endl;
   
//}


//void projectionToOpenGL( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
//{
 
//    float fu = K.at<double>(0,0);
//    float fv = (float)K.at<double>(1, 1);
//    float u0 = (float)K.at<double>(0, 2);
//    float v0 = (float)K.at<double>(1, 2);
    
//    float L = +(u0) * zNear / -fu;
//    float T = +(v0) * zNear / fv;
//    float R = -(w-u0) * zNear / -fu;
//    float B = -(h-v0) * zNear / fv;
    
        
//    p[0*4+0] = 2 * zNear / (R-L);
//    p[1*4+1] = 2 * zNear / (T-B);
//    p[2*4+2] = -(zFar +zNear) / (zFar - zNear);
//    p[2*4+0] = (R+L)/(R-L);
//    p[2*4+1] = (T+B)/(T-B);
//    p[2*4+3] = -1.0;
//    p[3*4+2] =  -(2*zFar*zNear)/(zFar-zNear);
    
//}

//void detectionToOpenGL(const Eigen::Matrix4f &M, double m[16], const cv::Mat &K, int imageWidth, int imageHeight,
//      float near, float far)
//{
//    //OpenGL has reversed Y & Z coords
//    Eigen::Matrix4f reverseYZ = Eigen::Matrix4f::Identity();
//    reverseYZ(0, 0) = 1.0;
//    reverseYZ(1, 1) = -1.0;
//    reverseYZ(2, 2) = -1.0;
    
//    //since we are in landscape mode
//    Eigen::Matrix4f rot2D = Eigen::Matrix4f::Identity();
//    rot2D(0, 0) = rot2D(1, 1) = 0;
//    rot2D(0, 1) =  1.0;
//    rot2D(1, 0) =  -1.0;
    
//    Eigen::Matrix4f projMat = Eigen::Matrix4f::Zero();
    
//    projMat(0, 0) =  2*(float)K.at<double>(0, 0)/imageWidth;
//    projMat(0, 2) =  -1 + (2*(float)K.at<double>(0, 2)/imageWidth);
//    projMat(1, 1) =  2*(float)K.at<double>(1, 1)/imageHeight;
//    projMat(1, 2) = -1 + (2*(float)K.at<double>(1, 2)/imageHeight);
//    projMat(2, 2) = -(far+near)/(far-near);
//    projMat(2, 3) = -2*far*near/(far-near);
//    projMat(3, 2) = -1;

//    std::cout << "OpenGL projection matrix  :" << std::endl << projMat << std::endl;

//    Eigen::Matrix4f mvMat = reverseYZ * M;
//    projMat = rot2D * projMat;

//    Eigen::Matrix4f mvp = projMat * mvMat;
//    std::cout << "matrix in normal format :" << std::endl << M << std::endl;

//    std::cout << "matrix in OpenGL  format :" << std::endl << mvp << std::endl;
//    //m = &mvp(0,0);
//    Eigen::Matrix4f R,mvpt;
//    mvpt = mvp.transpose();
//    R.block<3,3>(0,0) = mvpt.block<3,3>(0,0);
//    Eigen::Vector4f t = mvp.block<4,1>(0,3);
//   // std::cout << t <<std::endl;
//   // R.block<4,1>(0,3) = t;
//  //  std::cout << "matrix in OpenGL  format :" << std::endl << R << std::endl;
    
//     for (int i=0; i< 16; i++)
//          m[i] = (double) mvp(i);
//}


int getTimestamp(const cv::Mat &TS, double timestamp)
{
    
    cv::Mat res;
    cv::absdiff(TS, timestamp ,res);
    double minval; 
    int minind;
        
    cv::minMaxIdx(res, &minval,NULL, &minind);         
        
    return minind;
}

cv::Mat  readTimestamps(std::string filename)
{
    FILE *p = fopen(filename.c_str(),"r");
    char mystring [100];
    int cont = 0;
    while ( fgets (mystring , 100 , p)!=NULL)
        cont++;
    printf("The number of depth images is %d\n",cont);
    fclose(p);
    p = fopen(filename.c_str(),"r");
    cv::Mat_<double> timestamps(cont,1);
    double val; int ind;
    for (int i=0; i < cont; i++)    {
        fscanf(p,"%d",&ind);
        fscanf(p,"%lf",&val);        
        //printf("%d, %.0lf\n",ind, val);
        timestamps.at<double>(i) = val;
      // printf("%.0lf\n",timestamps.at<double>(i));
    }
    fclose(p);
    
    return timestamps;
}

int readStereoCalib(std::string calibFile, cv::Mat &K_depth, cv::Mat &dist_depth,cv::Mat &K_color, cv::Mat &dist_color, cv::Mat &R, cv::Mat &t){

    cv::FileStorage fs2(calibFile, cv::FileStorage::READ);
       
    fs2["cameraMatrixLeft"] >> K_color;
    fs2["cameraMatrixRight"] >> K_depth;
    
    fs2["distCoeffsLeft"] >> dist_color;
    fs2["distCoeffsRight"] >> dist_depth;
    
    fs2["RotationMatrix"] >> R;
    fs2["Translation"] >> t;


//    cv::transpose(R,R);
//t = -R*t;
    
    fs2.release();
    return 0;

}


int main(int argc, char **argv)
{
    Viewer pcl_viewer;

    // LOAD STUFF
    // Help
    bool help = false;
    if (pcl::console::find_argument(argc, argv,"-h") > 0)
    {
        std::cout << "HELP!" << std::endl;
        std::cout << "-f     sequence_directory" << std::endl;
        std::cout << "-c     calib_file" << std::endl;
        std::cout << "-s     start_img" << std::endl;
        std::cout << "-e     end_img" << std::endl;
        std::cout << "-m     model" << std::endl;
        std::cout << "-near  near" << std::endl;
        std::cout << "-far   far"  << std::endl;
        return 0;
    }

    // Load sequence directory
    std::string sequence_dir;
    pcl::console::parse_argument(argc, argv,"-f",sequence_dir);
    if (sequence_dir.empty())
        sequence_dir = "/media/DATA/wetlab_v1/f2000/";
    cv::Mat ts_color1,ts_depth1;
    ts_depth1 = readTimestamps(sequence_dir+"/depth/timestamps");
    ts_color1 = readTimestamps(sequence_dir+"/color/timestamps");

    // Load calibration file
    std::string scalib;
    pcl::console::parse_argument(argc, argv,"-c",scalib);
    if (scalib.empty())
        scalib = "/home/alejandro/workspace/colorPC/calibFiles/F200Calib_MRPT.yaml";
    cv::Mat K_depth(3,3,CV_64F),dist_depth(4,1,CV_64F),R(3,3,CV_64F);
    cv::Mat K_color(3,3,CV_64F),dist_color(4,1,CV_64F),t(3,1,CV_64F);

    readStereoCalib(scalib.c_str(),K_depth,dist_depth,K_color,dist_color,R,t);

    // Load start/end image numbers
    int img_start = 1;
    int img_end = 6500;
    pcl::console::parse_argument(argc, argv, "-s", img_start);
    pcl::console::parse_argument(argc, argv, "-e", img_end);

    // Load model
    std::string plyModel = "/home/alejandro/workspace/renderer/models/box.ply";
    pcl::console::parse_argument(argc,argv, "-m", plyModel);

    // Load near/far rage
    float near = 0.01;
    float far = 1000;
    pcl::console::parse_argument(argc, argv, "-near", near);
    pcl::console::parse_argument(argc, argv, "-far", far);

    // Load near/far rage
    float th1 = 0.2;
    float th2 = 20;
    pcl::console::parse_argument(argc, argv, "-th1", th1);
    pcl::console::parse_argument(argc, argv, "-th2", th2);

    // Select first image
    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(5) << img_start << ".png";
    cv::Mat img_depth = cv::imread(sequence_dir+"/depth/"+ss.str(),CV_LOAD_IMAGE_ANYDEPTH);
    img_depth.convertTo(img_depth,CV_32F);
    double timestampd1 = ts_depth1.at<double>(img_start);
    int imcolor1 = getTimestamp(ts_color1,timestampd1);
    std::ostringstream ss_color;
    ss_color << std::setfill('0') << std::setw(5) << imcolor1 << ".jpg";
    cv::Mat img_color = cv::imread(sequence_dir+"/color/"+ss_color.str());

    // Generate look-up table to rectify images
    cv::Size size_color(img_color.cols,img_color.rows);
    cv::Mat map_color_x, map_color_y;
    cv::initUndistortRectifyMap(K_color,dist_color,cv::Mat(),K_color,size_color,CV_32FC1,map_color_x,map_color_y);
    cv::Size size_depth(img_depth.cols,img_depth.rows);
    cv::Mat map_depth_x, map_depth_y;
    cv::initUndistortRectifyMap(K_depth,dist_depth,cv::Mat(),K_depth,size_depth,CV_32FC1,map_depth_x,map_depth_y);

    // Generate rectified images
    cv::remap(img_color,img_color,map_color_x,map_color_y,cv::INTER_NEAREST);
    cv::remap(img_depth,img_depth,map_depth_x,map_depth_y,cv::INTER_NEAREST);

    cv::Mat img_color_scaled;
    int scaling_factor = 2;
    cv::Size size(img_color.cols/scaling_factor,img_color.rows/scaling_factor);
    cv::resize(img_color,img_color_scaled,size);
    cv::Mat K_color_scaled = K_color/((double)scaling_factor);
    K_color_scaled.at<double>(2,2) = 1.0;

    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*1+1, 2*1+1 ),cv::Point( 1, 1 ) );

    std::vector<cv::Point2d> points2D;

//    // 1. Image cropper
//    cv::Mat img_to_click;
////    img_to_click = img_color;
////    cv::Mat img_edges;
////    img_edges = canny(img_color_scaled,th1,th2);
////    img_to_click = img_edges;
//    img_to_click = img_color_scaled;
//    std::string  src_window = "CROP";
//    ImageCropper imc(src_window,6);
//    cv::setMouseCallback(src_window.c_str(),&ImageCropper::mouseCallbackPoints,&imc);
//        imc.setImage(img_to_click);
//    imshow(src_window.c_str(),img_to_click);
//    cv::moveWindow(src_window.c_str(),0,0);
//    imc.getPoints(&points2D);
//    while (points2D.size() < 6)
//    {
//        imc.getPoints(&points2D);
//        cv::waitKey(2);
//    }

//    imc.getPoints(&points2D);
//    cv::destroyWindow(src_window.c_str());

    // 2. Hardcode the points
    // Start 1950
    points2D.push_back(cv::Point2d(534,436));
    points2D.push_back(cv::Point2d(607,428));
    points2D.push_back(cv::Point2d(600,527));
    points2D.push_back(cv::Point2d(534,533));
    points2D.push_back(cv::Point2d(505,300));
    points2D.push_back(cv::Point2d(562,294));

    for (int i = 0; i < 6; i++)
        points2D[i] *= scaling_factor;



//    ObjectRecognition objR1(K_depth,K_color,R,t);

    ORK_Renderer::Renderer3d renderer = ORK_Renderer::Renderer3d(plyModel);

    int width = img_color.cols;
    int height = img_color.rows;
    double P[16];
    projectionToOpenGLTopLeft(P,K_color, width , height,near,far);
    double focal_length_x = K_color.at<double>(0,0);
    double focal_length_y =  K_color.at<double>(1,1);

    renderer.set_parameters(width,height, focal_length_x, focal_length_y, near, far,P);


    std::vector<cv::Point3d> points3D;
    cv::Mat_<double> rvec, tvec,rotM;
    points3D = boxModel();
    cv::solvePnP(points3D,points2D,K_color,cv::Mat() ,rvec,tvec,false, CV_ITERATIVE); // CV_EPNP CV_ITERATIVE

    Rodrigues(rvec,rotM);

    //convert  to vectors
    double *r_ = (double *) rotM.data;
    double *t_ = (double *) tvec.data;

    Eigen::Vector4d quat = Eigen::Vector4d(0.642488, 0.580713, -0.349195, 0.357833);
    Eigen::Matrix3d rot_mat = quatToMatrix(quat);
    Eigen::Vector3d trans = Eigen::Vector3d(0.054173, 0.102074, 0.549953);
    double m[16] ={rot_mat(0,0),rot_mat(1,0),rot_mat(2,0),0.0,
                   rot_mat(0,1),rot_mat(1,1),rot_mat(2,1),0.0,
                   rot_mat(0,2),rot_mat(1,2),rot_mat(2,2),0.0,
                   trans(0), trans(1), trans(2), 1.0
    };

    double m2[16] = {r_[0],r_[3],r_[6],0.0,
                    r_[1],r_[4],r_[7],0.0,
                    r_[2],r_[5],r_[8],0.0,
                    t_[0], t_[1], t_[2], 1.0
    };

//    double m[16] = {1,0,0,0.0,
//                    0,1,0,0.0,
//                    0,0,1,0.0,
//                    0.056458, 0.106812, 0.584552, 1.0
//    };

    Eigen::Matrix4d M(m) ;
    Eigen::Matrix4d M2(m2) ;

    std::cout << "Pose from PnP : \n" << M2 << std::endl;
//    Eigen::Matrix3d rot_mat = M.block<3,3>(0,0);
//    Eigen::Matrix3d aux = rot_mat;
//    rot_mat = aux.transpose();
//    double roll = atan2(rot_mat(2,1), rot_mat(2,2));
//    double pitch = asin(rot_mat(2,0));
//    double yaw = -atan2(rot_mat(1,0), rot_mat(0,0));
//    std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
//    std::cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI << std::endl;

//    cv::Mat M_cv;
//    cv::eigen2cv(M,M_cv);
    pcl_viewer.snapPLYmodel(plyModel,M2, "model");
//    cv::eigen2cv(M2,M_cv);
    pcl_viewer.snapPLYmodel("/home/alejandro/workspace/renderer/models/yellow_box.ply",M, "model2");

    cv::Rect rect(0,0,width,height);

    renderer.lookAt(m);

    //    renderer.renderImageOnly(im_out,rect);
    //    cv::flip(im_out,im_out,0);
    //    cv::Mat projected;
    //    cv::addWeighted(im_out,1.0,img_color_scaled,0.5,0.0, projected);
    //    imshow("3D Model", projected);



    cv::Mat img_out_depth, mask_out, img_depth_model;
    renderer.renderDepthOnly(img_out_depth,mask_out,rect);
    cv::flip(mask_out,mask_out,0);
//    cv::erode(mask_out,mask_out,element);
//    cv::imwrite("mask.png",mask_out);
//    imshow("mask",mask_out);
//    cv::waitKey();
    cv::flip(img_out_depth,img_out_depth,0);
//    cv::imwrite("prof.png",img_out_depth);
    img_out_depth.convertTo(img_depth_model,CV_32F);
//    imshow("projected model", falseColor(img_depth_model));
//    cv::Mat projected;
//    cv::addWeighted(im_out,1.0,img_color_scaled,0.5,0.0, projected);
//    imshow("3D Model", projected);

    // Create cloud
//    cv::Mat model_normals = computeNormalMap(img_depth_model,K_color);
    cv::Mat model_normals = normalim(img_depth_model);
    cv::Mat model_normals_filtered;
//    cv::bilateralFilter(model_normals,model_normals_filtered, 5, 150, 150, cv::BORDER_DEFAULT );
    model_normals_filtered = model_normals;

//    imshow("normals", model_normals_filtered);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    model_cloud = compute3DpointsOrganized(img_depth_model,K_color);
//    pcl_viewer.drawCloud(model_cloud,0,255,0,1);

    cv::Mat img_model_edges;
//    img_edges2 = canny(model_normals_filtered,th1, th2);
//    img_model_edges = sobel(model_normals_filtered);
//    img_model_edges = canny_three_channels(model_normals_filtered,th1,th2*2);
    img_model_edges = edges_from_normals(model_normals_filtered,img_depth_model,10);
    cv::Mat img_model_edges_scaled;
    cv::resize(img_model_edges,img_model_edges_scaled,size);
//    imshow("edges1", img_model_edges);
//    thinning(img_model_edges_scaled);

//    imshow("edges2", img_model_edges_scaled);

    // Create cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr initial_normals (new pcl::PointCloud<pcl::Normal>);
    initial_cloud_color = compute3Dpoints(img_depth,K_depth,img_color,K_color,R,t);
    pcl::copyPointCloud(*initial_cloud_color,*initial_cloud);
    initial_normals = getNormals(initial_cloud,0.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_contour(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector4f plane_coeffs = getPlaneWithNormals(initial_cloud,initial_normals,table_contour);

    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_points;
    Eigen::Affine3f c2t = computeCamera2TableMatrix(initial_cloud, table_contour, initial_points);
    Eigen::Affine3f t2c = c2t.inverse();
    pcl::transformPointCloud(*initial_points,*initial_points,t2c);
    pcl_viewer.drawRectangle (initial_points, 255, 255, 255, 1, "table");

//    std::cout << c2t.matrix() << std::endl << std::endl;
//    std::cout << t2c.matrix() << std::endl << std::endl;

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr oriented_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr oriented_cloud (new pcl::PointCloud<pcl::PointXYZ>);

//    pcl::transformPointCloud(*initial_cloud_color,*oriented_cloud_color,c2t);
//    pcl::transformPointCloud(*initial_cloud,*oriented_cloud,c2t);

//    pcl_viewer.drawColorCloud(initial_cloud_color,1);
//    pcl_viewer.drawCloud(initial_cloud,255,0,0,2);

//    pcl_viewer.drawColorCloud(oriented_cloud_color,1);
//    pcl_viewer.drawCloud(oriented_cloud,255,0,0,2);

//    pcl_viewer.drawNormals(initial_normals_unorganized,initial_cloud_unorganized);
//    pcl_viewer.cloud_viewer_.spin();


    for (int i = img_start; i <= img_end; i++)
//    int i = img_start;
    {

        printf("frame %d/%d\n", i, img_end);

        //Load depth image
        std::ostringstream ss;
        ss << std::setfill('0') << std::setw(5) << i << ".png";
        img_depth = cv::imread(sequence_dir+"/depth/"+ss.str(),CV_LOAD_IMAGE_ANYDEPTH);
        img_depth.convertTo(img_depth,CV_32F);

        //Load the closest color image
        double timestampd1 = ts_depth1.at<double>(i);
        int imcolor1 = getTimestamp(ts_color1,timestampd1);
        std::ostringstream ss_color;
        ss_color << std::setfill('0') << std::setw(5) << imcolor1 << ".jpg";
        img_color = cv::imread(sequence_dir+"/color/"+ss_color.str());

        // Undistort
        cv::remap(img_color,img_color,map_color_x,map_color_y,cv::INTER_NEAREST);
        cv::remap(img_depth,img_depth,map_depth_x,map_depth_y,cv::INTER_NEAREST);

        cv::resize(img_color,img_color_scaled,size);

        cv::Mat img_edges = canny(img_color_scaled,th1,th2);
//        img_edges = sobel(img_color_scaled);
//        imshow("edges", img_edges);


        cv::Mat img_contours(img_model_edges_scaled.rows,img_model_edges_scaled.cols,CV_8UC3,cv::Scalar(0,0,0));
        img_color_scaled.copyTo(img_contours);//,img_edges);
        cv::Mat img_blue(img_model_edges_scaled.rows,img_model_edges_scaled.cols,CV_8UC3,cv::Scalar(255,0,0));
        img_blue.copyTo(img_contours,img_edges);
        cv::Mat img_orange(img_model_edges_scaled.rows,img_model_edges_scaled.cols,CV_8UC3,cv::Scalar(0,100,255));
        img_orange.copyTo( img_contours, img_model_edges_scaled);
        cv::Mat img_green(img_model_edges_scaled.rows,img_model_edges_scaled.cols,CV_8UC3,cv::Scalar(0,255,0));
        cv::Mat overlap;
        cv::bitwise_and(img_model_edges_scaled,img_edges,overlap);

        cv::dilate(overlap,overlap,element);
        img_green.copyTo( img_contours, overlap);

//        std::vector<std::vector<cv::Point> > contours;
//        std::vector<cv::Vec4i> hierarchy;

//        /// Find contours
//        cv::findContours( img_model_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

//        /// Draw contours
//        cv::Mat drawing = cv::Mat::zeros( img_edges.size(), CV_8UC3 );
//        for( int i = 0; i< contours.size(); i++ )
//        {
//            cv::Scalar color = cv::Scalar(0,255,0 );
//            cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
//        }


        imshow("edges superposed", img_contours);


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud_color = compute3Dpoints(img_depth,K_depth, img_color_scaled, K_color_scaled, R, t);
        pcl::copyPointCloud(*cloud_color,*cloud);

        pcl_viewer.drawColorCloud(cloud_color,1);



        if(img_depth.empty() || img_color.empty())
        {
            printf("ERROR: loading image \"%s\" failed!\n", ss.str().c_str());
            return 1;
        }

        pcl_viewer.cloud_viewer_.spinOnce();
    }

    pcl_viewer.cloud_viewer_.spin();
    return 0;
}

