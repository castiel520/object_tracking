#include <stdio.h>
#include <iomanip>
#include <string>
#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <boost/timer.hpp>

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
#include <pcl/io/vtk_lib_io.h>
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
#include "background.h"
#include "../../PWP3D/PerseusLib/PerseusLib.h"
//#include <PWP3D/PerseusLib.h>
//#include "../../PWP3D/install/include/PWP3D/PerseusLib.h"


#include <GL/glut.h>


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

std::vector<cv::Point2d>  getPointsForPnP(cv::Mat img)
{
    std::vector<cv::Point2d> points2D;

//    // 1. Image cropper
//    cv::Mat img_to_click;
//    img_to_click = img;
////    cv::Mat img_edges;
////    img_edges = canny(img_color_scaled,th1,th2);
////    img_to_click = img_edges;
//    std::string  src_window = "CROP";
//    ImageCropper imc(src_window,6);
//    cv::setMouseCallback(src_window.c_str(),&ImageCropper::mouseCallbackPoints,&imc);
//    imc.setImage(img_to_click);
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

//    // 2. Hardcode the points
//    // Start 1980
//    points2D.push_back(cv::Point2d(534,436));
//    points2D.push_back(cv::Point2d(607,428));
//    points2D.push_back(cv::Point2d(600,527));
//    points2D.push_back(cv::Point2d(534,533));
//    points2D.push_back(cv::Point2d(505,300));
//    points2D.push_back(cv::Point2d(562,294));
//    // Start 4675
//    points2D.push_back(cv::Point2d(821,413));
//    points2D.push_back(cv::Point2d(881,399));
//    points2D.push_back(cv::Point2d(856,492));
//    points2D.push_back(cv::Point2d(793,503));
//    points2D.push_back(cv::Point2d(702,291));
//    points2D.push_back(cv::Point2d(749,284));
    // Start 18010
    points2D.push_back(cv::Point2d(438,400));
    points2D.push_back(cv::Point2d(444,362));
    points2D.push_back(cv::Point2d(452,452));
    points2D.push_back(cv::Point2d(447,496));
    points2D.push_back(cv::Point2d(199,398));
    points2D.push_back(cv::Point2d(228,357));

    for (int i = 0; i < 6; i++)
        points2D[i] *= 2;//scaling_factor;

    return points2D;

}

Eigen::Matrix4d getPoseFromPnP(ORK_Renderer::Renderer3d &renderer, int width, int height, cv::Mat K_color, float near, float far, std::vector<cv::Point2d> points2D)
{
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


    double m[16] = {r_[0],r_[3],r_[6],0.0,
                    r_[1],r_[4],r_[7],0.0,
                    r_[2],r_[5],r_[8],0.0,
                    t_[0], t_[1], t_[2], 1.0
    };

    Eigen::Matrix4d M(m) ;
//        std::cout << "Pose from PnP : \n" << M << std::endl;


    return M;
}

void generateModelImagesFromPose(ORK_Renderer::Renderer3d & renderer,Eigen::Matrix4d M, cv::Mat & img_depth_model, cv::Mat & mask_out)
{
    double m[16] = {M(0,0),M(1,0),M(2,0),0.0,
                    M(0,1),M(1,1),M(2,1),0.0,
                    M(0,2),M(1,2),M(2,2),0.0,
                    M(0,3),M(1,3),M(2,3),1.0
    };

    cv::Mat img_out_depth;

    renderer.lookAt(m);
    renderer.renderDepthOnly(img_out_depth,mask_out);

//    imshow("depz",falseColor(img_out_depth));

    cv::flip(img_out_depth,img_out_depth,0);
    cv::flip(mask_out,mask_out,0);
//    cv::imwrite("mask.png",mask_out);
    img_out_depth.convertTo(img_depth_model,CV_32F);
}

void computeEdgesFromNormalMap(cv::Mat img_depth_model, cv::Mat & model_normals, cv::Mat & img_model_edges)
{
   model_normals = normalim(img_depth_model);
    cv::Mat model_normals_filtered;
    //    cv::bilateralFilter(model_normals,model_normals_filtered, 5, 150, 150, cv::BORDER_DEFAULT );
    model_normals_filtered = model_normals;
    img_model_edges = edges_from_normals(model_normals_filtered,img_depth_model,10);
    //    thinning(img_model_edges_scaled);
}

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
    boost::timer timer;
    double duration;

//    Viewer pcl_viewer;

    // LOAD STUFF
    // Help
//    bool help = false;
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

    // Load background
    cv::Mat img_bg = cv::imread("/home/alejandro/Models/background.jpg");

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
    cv::remap(img_bg,img_bg,map_color_x,map_color_y,cv::INTER_NEAREST);

    // Set scaling
    int scaling_factor = 2;
    int width = img_color.cols;
    int height = img_color.rows;
    int width_scaled = width/scaling_factor;
    int height_scaled = height/scaling_factor;
    cv::Size size(width_scaled,height_scaled);
    cv::Mat K_color_scaled = K_color/((double)scaling_factor);
    K_color_scaled.at<double>(2,2) = 1.0;
    cv::Mat img_color_scaled;
    cv::resize(img_color,img_color_scaled,size);
    cv::Mat img_bg_scaled;
    cv::resize(img_bg,img_bg_scaled,size);

    cv::Mat img_contours(img_color.rows,img_color.cols,CV_8UC3,cv::Scalar(0,0,0));
    img_color.copyTo(img_contours);//,img_edges);

    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*1+1, 2*1+1 ),cv::Point( 1, 1 ) );

    std::vector<cv::Point2d> points2D;
    points2D = getPointsForPnP(img_color_scaled);
    ORK_Renderer::Renderer3d renderer = ORK_Renderer::Renderer3d(plyModel);
    Eigen::Matrix4d M = getPoseFromPnP(renderer, width, height, K_color, near, far, points2D);

//    Eigen::Matrix3d rot_mat = M.block<3,3>(0,0);
//    Eigen::Matrix3d aux = rot_mat;
//    rot_mat = aux.transpose();
//    double roll = atan2(rot_mat(2,1), rot_mat(2,2));
//    double pitch = asin(rot_mat(2,0));
//    double yaw = -atan2(rot_mat(1,0), rot_mat(0,0));
//    std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
//    std::cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI << std::endl;

//    pcl_viewer.snapPLYmodel(plyModel,M, "model");
//    cv::eigen2cv(M2,M_cv);
//    pcl_viewer.snapPLYmodel("/home/alejandro/workspace/renderer/models/yellow_box.ply",M_cv, "model2");

//    pcl_viewer.cloud_viewer_.spin();
    cv::Mat mask_out = cv::Mat(height,width,CV_8U,cv::Scalar(0));
    cv::Mat img_depth_model;
    generateModelImagesFromPose(renderer, M, img_depth_model,mask_out);
    cv::Mat model_normals_initial, img_model_edges_initial;
    computeEdgesFromNormalMap(img_depth_model, model_normals_initial,img_model_edges_initial);
    cv::Mat img_model_edges_scaled_initial;
    cv::resize(img_model_edges_initial,img_model_edges_scaled_initial,size);

    cv::Mat img_orange(img_model_edges_initial.rows,img_model_edges_initial.cols,CV_8UC3,cv::Scalar(0,100,255));
    img_orange.copyTo( img_contours, img_model_edges_initial);

    imshow("edges superposed", img_contours);
//    cv::waitKey();

    // Create cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    initial_cloud_color = compute3Dpoints(img_depth,K_depth,img_color,K_color,R,t);
    pcl::copyPointCloud(*initial_cloud_color,*initial_cloud);

    BackGround bg(initial_cloud);
//    pcl_viewer.drawRectangle (bg.vertex2c, 255, 255, 255, 1, "table");



    std::string sModelPath = "/home/alejandro/Models/orange_box.obj";
//    std::string sModelPath = "/home/alejandro/Models/orange_prism.obj";
//    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFileOBJ(sModelPath,mesh);
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
//    pcl::transformPointCloud(cloud, cloud, M);
//    pcl::toPCLPointCloud2(cloud, mesh.cloud);
//    pcl_viewer.cloud_viewer_.addPolygonMesh(mesh,"meshes",0);

//    std::string sSrcImage = "/media/DATA/wetlab_v1/f2000/color/01950.jpg";
//    std::string sCameraMatrix = "/home/alejandro/workspace/PWP3D/Files/CameraCalibration/f2000.cal";
//    std::string sTargetMask = "/home/alejandro/Models/target_mask.png";
    cv::Mat target_mask(height_scaled,width_scaled,CV_8U,cv::Scalar(255));
    // Segment color image in HS space
    cv::Mat img_color_hsv, img_color_h, img_color_s;
    cv::cvtColor(img_color_scaled, img_color_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(img_color_hsv, channels);
    img_color_h = channels[0]; img_color_h.convertTo(img_color_h,CV_32F);
    img_color_s = channels[1]; img_color_s.convertTo(img_color_s,CV_32F);
    cv::Mat image_dif_h = (90 - abs(abs(img_color_h-142)-90))/179;
    cv::Mat image_dif_s = abs(img_color_s-127)/255;
    cv::Mat image_dif = (0.9*image_dif_h + 0.1*image_dif_s);
//    target_mask = (image_dif > 0.1)*255;
    cv::Mat hand_mask_scaled(height,width,CV_8U,cv::Scalar(255));
    hand_mask_scaled = (image_dif < 0.1)*255;

    cv::Mat img_bg_hsv, img_bg_h, img_bg_s;
    cv::cvtColor(img_bg_scaled, img_bg_hsv, cv::COLOR_BGR2HSV);
    split(img_bg_hsv, channels);
    img_bg_h = channels[0]; img_bg_h.convertTo(img_bg_h,CV_32F);
    img_bg_s = channels[1]; img_bg_s.convertTo(img_bg_s,CV_32F);
    image_dif_h = (90 - abs(abs(img_bg_h-img_color_h)-90))/179;
    image_dif_s = abs(img_bg_s-img_color_s)/255;
    image_dif = (0.3*image_dif_h + 0.7*image_dif_s);
    cv::Mat mask_bg_scaled = image_dif < 0.1;
    cv::Mat mask_bg;
    cv::resize(mask_bg_scaled,mask_bg,size*2);
    cv::Mat mask_fg, mask_fg_scaled;
    mask_fg = 255-mask_bg;
    cv::resize(mask_fg,mask_fg_scaled,size);

    cv::Mat mask_out_scaled;
    cv::resize(mask_out,mask_out_scaled,size);
    cv::erode(mask_out,mask_out,element);
//    mask_out = 255 - mask_out;
//    mask_out = cv::imread("/home/alejandro/Models/mask.png");
//    std::cout << mask_out << std::endl;

//    imshow("mask",mask_out);

    int viewCount = 1, objectCount = 1;
    int objectId = 0, viewIdx = 0, objectIdx = 0;

    //result visualisation
    ImageUChar4* ResultImage = new ImageUChar4(width_scaled, height_scaled);

    ImageUChar4* camera = new ImageUChar4(width_scaled, height_scaled);
//        ImageUtils::Instance()->LoadImageFromFile(camera, (char*)sSrcImage.c_str());
    cv::Mat img_color4C = cv::Mat(height_scaled,width_scaled,CV_8UC4);
//    int from_to[] = { 0,0, 1,1, 2,2,};
//    cv::mixChannels(&img_color,2,&img_color4C,1,from_to,3);
    std::vector<cv::Mat> rgbChannels(4);
    cv::split(img_color_scaled, rgbChannels);
    cv::Mat alpha_mask(height_scaled,width_scaled,CV_8U,cv::Scalar(255));
    rgbChannels.push_back(alpha_mask);
    cv::merge(rgbChannels, img_color4C);
    ImageUtils::Instance()->LoadImageFromCVMat(camera, img_color4C);

    Object3D **objects = new Object3D*[objectCount];
    objects[objectIdx] = new Object3D(objectId, viewCount, (char*)sModelPath.c_str(), width_scaled, height_scaled);

    View3D **views = new View3D*[viewCount];
//    views[viewIdx] = new View3D(0, (char*)sCameraMatrix.c_str(), width, height);
    views[viewIdx] = new View3D(viewIdx, width_scaled, height_scaled, K_color_scaled.at<double>(0,0), K_color_scaled.at<double>(1,1), K_color_scaled.at<double>(0,2), height_scaled - K_color_scaled.at<double>(1,2),width_scaled, height_scaled,near,far);

//    ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
//                                              (char*)sTargetMask.c_str());
    ImageUtils::Instance()->LoadImageFromCVMat(views[viewIdx]->videoMask,
                                              target_mask);

        cv::Mat hand_mask;
    //    cv::resize(hand_mask_scaled,hand_mask,size*2);
    ////    cv::bitwise_or(hand_mask_scaled,mask_out_scaled,mask_out_scaled);

    // Load histogram from first image
    ImageUtils::Instance()->LoadImageFromCVMat(objects[objectIdx]->histSources[viewIdx], img_color4C);
    mask_out_scaled /= 255;
    mask_out_scaled = mask_out_scaled*(objectIdx+1);
    ImageUtils::Instance()->LoadImageFromCVMat(objects[objectIdx]->histMasks[viewIdx],mask_out_scaled);

//    // Load histogram from pre-selected image
//    std::string sHistSrc = "/media/DATA/wetlab_v1/f2000/color/01980.jpg";
//    std::string sHistMask = "/home/alejandro/Models/mask.png";
//    ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
//                                              (char*)sHistSrc.c_str());
//    ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
//                                              (char*)sHistMask.c_str(), objectIdx+1);

    HistogramEngine::Instance()->UpdateVarBinHistogram(
          objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
          objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);

    IterationConfiguration *iterConfig = new IterationConfiguration();
    iterConfig->width = width_scaled; iterConfig->height = height_scaled;
    iterConfig->iterViewIds[viewIdx] = 0;
    iterConfig->iterObjectCount[viewIdx] = 1;
    iterConfig->levelSetBandSize = 2;
    iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;
    iterConfig->iterViewCount = 1;
    iterConfig->iterCount = 1;
    iterConfig->useCUDAEF = true;
    iterConfig->useCUDARender = true;
//    iterConfig->useCUDAEF = false;
//    iterConfig->useCUDARender = false;

//    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.01f, 0.01f, 0.01f);
    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.05f, 0.0005f, 0.0005f, 0.0005f);
//    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.01f, 0.0001f, 0.0001f, 0.0001f);
//    float step_size = 10/((K_color.at<float>(0,0)+K_color.at<float>(1,1))/2);
//    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.5f, step_size,step_size, step_size);

    VFLOAT *rot;
    rot =(VFLOAT [9]) {M(0,0),M(0,1),M(0,2),M(1,0),M(1,1),M(1,2),M(2,0),M(2,1),M(2,2)};
    objects[objectIdx]->initialPose[viewIdx]->SetFrom(
            M(0,3), M(1,3), M(2,3), rot);

    OptimisationEngine::Instance()->Initialise(width_scaled, height_scaled);
    OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

    //result plot
    VisualisationEngine::Instance()->GetImage(
          ResultImage, GETIMAGE_PROXIMITY,
          objects[objectIdx], views[viewIdx], objects[objectIdx]->initialPose[viewIdx]);

    //result save to file
//    ImageUtils::Instance()->SaveImageToFile(ResultImage, str);
    cv::Mat InitMat(height_scaled,width_scaled,CV_8UC4, ResultImage->pixels);
//    cv::resize(InitMat,InitMat,size);
    imshow("edges superposed", InitMat);
//    cv::waitKey();

    OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);

    Eigen::Vector4d quat = Eigen::Vector4d(objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
                                           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
                                           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
                                           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);
    Eigen::Matrix3d rot_mat = quatToMatrix(quat);
    Eigen::Vector3d trans = Eigen::Vector3d(objects[objectIdx]->pose[viewIdx]->translation->x,
                                            objects[objectIdx]->pose[viewIdx]->translation->y,
                                            objects[objectIdx]->pose[viewIdx]->translation->z);
    double m2[16] ={rot_mat(0,0),rot_mat(1,0),rot_mat(2,0),0.0,
                    rot_mat(0,1),rot_mat(1,1),rot_mat(2,1),0.0,
                    rot_mat(0,2),rot_mat(1,2),rot_mat(2,2),0.0,
                    trans(0), trans(1), trans(2), 1.0
                   };



    //result plot
    VisualisationEngine::Instance()->GetImage(
          ResultImage, GETIMAGE_PROXIMITY,
          objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

    //result save to file
//    ImageUtils::Instance()->SaveImageToFile(ResultImage, str);
    cv::Mat ResultMat(height_scaled,width_scaled,CV_8UC4, ResultImage->pixels);
//    cv::resize(ResultMat,ResultMat,size);
    imshow("edges superposed", ResultMat);
//    cv::waitKey();

    Eigen::Matrix4d M2(m2) ;
//    pcl_viewer.snapPLYmodel("/home/alejandro/workspace/renderer/models/yellow_box.ply",M2, "model2");

//    std::cout << M << std::endl << std::endl;
//    std::cout << M2 << std::endl;

//    pcl::io::loadPolygonFileOBJ(sModelPath,mesh);
//    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
//    pcl::transformPointCloud(cloud, cloud, M2);
//    pcl::toPCLPointCloud2(cloud, mesh.cloud);
//    pcl_viewer.cloud_viewer_.addPolygonMesh(mesh,"meshes2",0);

//    Eigen::Matrix4d M2;
//    M2 = M;

    generateModelImagesFromPose(renderer, M2, img_depth_model,mask_out);
//    imshow("depth",falseColor(img_depth_model));
//    imshow("mask",mask_out);
//    cv::waitKey();
    cv::Mat model_normals, img_model_edges;
    computeEdgesFromNormalMap(img_depth_model, model_normals,img_model_edges);
    cv::Mat img_model_edges_scaled;
    cv::resize(img_model_edges,img_model_edges_scaled,size);

    cv::Mat prev_img_color4C_scaled;
    img_color4C.copyTo(prev_img_color4C_scaled);

    for (int i = img_start; i <= img_end; i++)
//    int i = img_start+2;
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

        cv::Mat img_edges = canny(img_color,th1,th2);
//        img_edges = sobel(img_color_scaled);
//        imshow("edges", img_edges);

//////////////////////ç//////////////////////ç//////////////////////ç//////////////////////ç



        cv::cvtColor(img_color_scaled, img_color_hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> channels;
        split(img_color_hsv, channels);
        img_color_h = channels[0];
        img_color_s = channels[1];
        img_color_h.convertTo(img_color_h,CV_32F);
        img_color_s.convertTo(img_color_s,CV_32F);
        image_dif_h = (90 - abs(abs(img_color_h-142)-90))/179;
        image_dif_s = abs(img_color_s-127)/255;
        image_dif = (0.9*image_dif_h + 0.1*image_dif_s);
        target_mask = (image_dif > 0.1)*255;
        hand_mask_scaled = (image_dif < 0.1)*255;
        cv::resize(hand_mask_scaled,hand_mask,size*2);

        image_dif_h = (90 - abs(abs(img_bg_h-img_color_h)-90))/179;
        image_dif_s = abs(img_bg_s-img_color_s)/255;
        image_dif = (0.9*image_dif_h + 0.1*image_dif_s);
        mask_bg_scaled = image_dif < 0.05;
        cv::resize(mask_bg_scaled,mask_bg,size*2);
        mask_fg = 255-mask_bg;
        cv::resize(mask_fg,mask_fg_scaled,size);

//        cv::Mat target_mask_scaled;
//        cv::resize(target_mask,target_mask_scaled,size);
//        cv::bitwise_and(target_mask_scaled,mask_out_scaled,mask_out_scaled);
//        cv::Mat overlap_mask, overlap_mask_scaled;
//        cv::bitwise_and(mask_out_scaled,hand_mask_scaled,overlap_mask_scaled);
//        cv::resize(overlap_mask_scaled,overlap_mask,size*2);
//        overlap_mask /= 255;
//        overlap_mask = overlap_mask*(objectIdx+1);

//        cv::Mat img_black(img_color_scaled.rows,img_color_scaled.cols,CV_8UC3,cv::Scalar(0,0,0));
//        img_black.copyTo(img_color_scaled,overlap_mask_scaled);
        cv::Mat img_gray(img_color_scaled.rows,img_color_scaled.cols,CV_8UC3,cv::Scalar(0,0,1));
        img_gray.copyTo(img_color_scaled,hand_mask_scaled);
//        img_black.copyTo(img_color_scaled,hand_mask_scaled);
        cv::Mat img_white(img_color_scaled.rows,img_color_scaled.cols,CV_8UC3,cv::Scalar(255,255,255));
        img_white.copyTo(img_color_scaled,mask_bg_scaled);


        cv::split(img_color_scaled, rgbChannels);
        cv::Mat alpha_mask(height_scaled,width_scaled,CV_8U,cv::Scalar(255));
        rgbChannels.push_back(alpha_mask);
        cv::merge(rgbChannels, img_color4C);
        ImageUtils::Instance()->LoadImageFromCVMat(camera, img_color4C);

//        cv::bitwise_or(mask_out_scaled,overlap_mask_scaled,mask_out_scaled);
//        cv::Mat img_histogram;
//        img_histogram = prev_img_color4C_scaled;
//        img_color4C.copyTo(img_histogram,hand_mask_scaled);

//        ImageUtils::Instance()->LoadImageFromCVMat(objects[objectIdx]->histSources[viewIdx], img_color4C);
//        imshow("mask",mask_out_scaled);
//        mask_out_scaled /= 255;
//        mask_out_scaled = mask_out_scaled*(objectIdx+1);

//        ImageUtils::Instance()->LoadImageFromCVMat(objects[objectIdx]->histMasks[viewIdx],mask_out_scaled);

//        HistogramEngine::Instance()->UpdateVarBinHistogram(
//              objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
//              objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);
//        ImageUtils::Instance()->LoadImageFromCVMat(views[viewIdx]->videoMask,
//                                                  mask_fg);
//        imshow("maskara",target_mask);




        OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

        Eigen::Matrix4d prev_M = M2;

        int i= 0;
//        for (i=0; i<100; i++)
        bool loop = false;
        timer.restart();

        while (i<200)
        {
            OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);

            objects[objectIdx]->pose[viewIdx]->CopyInto(objects[objectIdx]->initialPose[viewIdx]);

            quat = Eigen::Vector4d(objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
                                   objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
                                   objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
                                   objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);
            rot_mat = quatToMatrix(quat);
            trans = Eigen::Vector3d(objects[objectIdx]->pose[viewIdx]->translation->x,
                                    objects[objectIdx]->pose[viewIdx]->translation->y,
                                    objects[objectIdx]->pose[viewIdx]->translation->z);


            //        M2=Eigen::Matrix4d(m2) ;
            M2.block<3,3>(0,0) = rot_mat;
            M2.block<3,1>(0,3) = trans;

            //        M2(0,3) = M2(0,3)+0.005;

//            std::cout << "M2: \n" << M2 << std::endl;

            Eigen::Matrix4d M_diff;
            M_diff = prev_M.inverse()*M2;

            float x,y,z,roll,pitch,yaw;
            Eigen::Affine3f matrix_aux = Eigen::Translation3f(M_diff.block<3,1>(0,3).cast<float>()) * Eigen::AngleAxisf(M_diff.block<3,3>(0,0).cast<float>());
            pcl::getTranslationAndEulerAngles (matrix_aux, x, y, z, roll, pitch, yaw);

            float thresh_t = 0.001;
            float thresh_r = 0.0001;
            if ((fabs(x)<thresh_t)&&(fabs(y)<thresh_t)&&(fabs(z)<thresh_t)&&(fabs(roll)<thresh_r)&&(fabs(pitch)<thresh_r)&&(fabs(yaw)<thresh_r))
                break;
            else
            {
                if ((i==199) && (loop == false))
                {
//                    std::cout << i << std::endl;

                    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.01f, 0.001f, 0.001f, 0.001f);
                    i = 0;
                    loop = true;
                }
            }
//            std::cout << "X: " << x << std::endl;
//            std::cout << "Y: " << y << std::endl;
//            std::cout << "Z: " << z << std::endl;

//            std::cout << "Roll: " << roll*180/M_PI << std::endl;
//            std::cout << "Pitch: " << pitch*180/M_PI << std::endl;
//            std::cout << "Yaw: " << yaw*180/M_PI << std::endl;

            prev_M = M2;

            i = i+1;
        }
        std::cout << timer.elapsed() << std::endl;

        objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.05f, 0.0005f, 0.0005f, 0.0005f);

//        std::cout << i << std::endl;

        generateModelImagesFromPose(renderer, M2, img_depth_model,mask_out);
        cv::resize(mask_out,mask_out_scaled,size);
        computeEdgesFromNormalMap(img_depth_model, model_normals,img_model_edges);



//        pcl_viewer.cloud_viewer_.removeShape("model");
//        pcl_viewer.snapPLYmodel(plyModel,M2, "model");



///////////////////////////////////////////////////////////////////////////////////////////////////






        img_color.copyTo(img_contours);//,img_edges);
        cv::Mat img_blue(img_model_edges.rows,img_model_edges.cols,CV_8UC3,cv::Scalar(255,0,0));
        img_blue.copyTo(img_contours,img_edges);
        cv::Mat img_yellow(img_model_edges.rows,img_model_edges.cols,CV_8UC3,cv::Scalar(0,255,255));
        img_yellow.copyTo( img_contours, img_model_edges);
//        cv::Mat img_orange(img_model_edges.rows,img_model_edges.cols,CV_8UC3,cv::Scalar(0,100,255));
//        img_orange.copyTo( img_contours, img_model_edges_initial);
        cv::Mat img_green(img_model_edges.rows,img_model_edges.cols,CV_8UC3,cv::Scalar(0,255,0));
        cv::Mat overlap;
        cv::bitwise_and(img_model_edges,img_edges,overlap);
        cv::dilate(overlap,overlap,element);
        img_green.copyTo( img_contours, overlap);
//        cv::Mat img_purple(img_model_edges.rows,img_model_edges.cols,CV_8UC3,cv::Scalar(255,0,180));
//        img_purple.copyTo(img_contours,mask_bg);

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


//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
//        cloud_color = compute3Dpoints(img_depth,K_depth, img_color_scaled, K_color_scaled, R, t);
//        pcl::copyPointCloud(*cloud_color,*cloud);
//        pcl_viewer.drawColorCloud(cloud_color,1);

//        pcl_viewer.cloud_viewer_.spinOnce();


        if(img_depth.empty() || img_color.empty())
        {
            printf("ERROR: loading image \"%s\" failed!\n", ss.str().c_str());
            return 1;
        }

        img_color4C.copyTo(prev_img_color4C_scaled);
//        cv::waitKey();
    }

//    pcl_viewer.cloud_viewer_.spin();
    return 0;
}

