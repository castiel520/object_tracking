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

std::vector<cv::Point3d> cylinderModel()
{
    std::vector<cv::Point3d> points;
    cv::Point3d p;
    p = cv::Point3f(0.041575,0.017221,0.0075);
    points.push_back(p);
    p = cv::Point3f(-0.041575,-0.017221,0.0075);
    points.push_back(p);
    p = cv::Point3f(0.017221,-0.041575,0.0075);
    points.push_back(p);
    p = cv::Point3f(-0.017221,0.041575,0.0075);

    points.push_back(p);

    return points;
}

std::vector<cv::Point2d>  getPointsForPnP(cv::Mat img, int scale)
{
    std::vector<cv::Point2d> points2D;

    // 1. Image cropper
    cv::Mat img_to_click;
    img_to_click = img;
//    cv::Mat img_edges;
//    img_edges = canny(img_color_scaled,th1,th2);
//    img_to_click = img_edges;
    std::string  src_window = "CROP";
    ImageCropper imc(src_window,6);
    cv::setMouseCallback(src_window.c_str(),&ImageCropper::mouseCallbackPoints,&imc);
    imc.setImage(img_to_click);
    imshow(src_window.c_str(),img_to_click);
    cv::moveWindow(src_window.c_str(),0,0);
    imc.getPoints(&points2D);
    while (points2D.size() < 4)
    {
        imc.getPoints(&points2D);
        cv::waitKey(2);
    }

//    imc.getPoints(&points2D);
//    cv::destroyWindow(src_window.c_str());

//    // 2. Hardcode the points
//    // Start 1980
//    scale = 2;
//    points2D.push_back(cv::Point2d(534,436));
//    points2D.push_back(cv::Point2d(607,428));
//    points2D.push_back(cv::Point2d(600,527));
//    points2D.push_back(cv::Point2d(534,533));
//    points2D.push_back(cv::Point2d(505,300));
//    points2D.push_back(cv::Point2d(562,294));
//    // Start 4675
//    scale = 2;
//    points2D.push_back(cv::Point2d(821,413));
//    points2D.push_back(cv::Point2d(881,399));
//    points2D.push_back(cv::Point2d(856,492));
//    points2D.push_back(cv::Point2d(793,503));
//    points2D.push_back(cv::Point2d(702,291));
//    points2D.push_back(cv::Point2d(749,284));
//    // Start 18010
//    scale = 2;
//    points2D.push_back(cv::Point2d(438,400));
//    points2D.push_back(cv::Point2d(444,362));
//    points2D.push_back(cv::Point2d(452,452));
//    points2D.push_back(cv::Point2d(447,496));
//    points2D.push_back(cv::Point2d(199,398));
//    points2D.push_back(cv::Point2d(228,357));

    for (int i = 0; i < 4; i++)
        points2D[i] *= scale;//scaling_factor;

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
//    points3D = boxModel();
    points3D = cylinderModel();
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
//    Viewer pcl_viewer;

    /**********************************************************
     *********************   LOAD STUFF   *********************
     **********************************************************/

    // Help
    if (pcl::console::find_argument(argc, argv,"-h") > 0)
    {
        std::cout << "HELP!" << std::endl;
        std::cout << "-f        sequence_directory" << std::endl;
        std::cout << "-c        calib_file" << std::endl;
        std::cout << "-s        start_img" << std::endl;
        std::cout << "-e        end_img" << std::endl;
        std::cout << "-md       model directory" << std::endl;
        std::cout << "-mn       model name" << std::endl;
        std::cout << "-near     near" << std::endl;
        std::cout << "-far      far"  << std::endl;
        std::cout << "-th1      canny_threshold1" << std::endl;
        std::cout << "-th2      canny_threshold2" << std::endl;
        std::cout << "-bg       background_image" << std::endl;
        std::cout << "-scale    scaling_factor" << std::endl;
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
        scalib = "/home/alejandro/workspace/renderer/calibFiles/F200Calib_MRPT.yaml";
    cv::Mat K_depth(3,3,CV_64F),dist_depth(4,1,CV_64F),R(3,3,CV_64F);
    cv::Mat K_color(3,3,CV_64F),dist_color(4,1,CV_64F),t(3,1,CV_64F);

    readStereoCalib(scalib.c_str(),K_depth,dist_depth,K_color,dist_color,R,t);

    // Load start/end image numbers
    int img_start = 1;
    int img_end = 6500;
    pcl::console::parse_argument(argc, argv, "-s", img_start);
    pcl::console::parse_argument(argc, argv, "-e", img_end);

    // Load model directory and name
    std::string modelDirectory = "/home/alejandro/Models/";
    std::string modelName = "orange_box";
    pcl::console::parse_argument(argc,argv, "-md", modelDirectory);
    pcl::console::parse_argument(argc,argv, "-mn", modelName);
    std::string plyModel = modelDirectory + modelName + ".ply";
    std::string objModel = modelDirectory + modelName + ".obj";

    // Load near/far rage
    float near = 0.01;
    float far = 1000;
    pcl::console::parse_argument(argc, argv, "-near", near);
    pcl::console::parse_argument(argc, argv, "-far", far);

    // Load canny thresholds
    float th1 = 70;
    float th2 = 80;
    pcl::console::parse_argument(argc, argv, "-th1", th1);
    pcl::console::parse_argument(argc, argv, "-th2", th2);

    // Load background
    std::string backgroundFile = "/home/alejandro/Models/background.jpg";
    pcl::console::parse_argument(argc, argv, "-bg", backgroundFile);;
    cv::Mat img_bg = cv::imread(backgroundFile);

    // Load scaling factor
    int scaling_factor = 2;
    pcl::console::parse_argument(argc,argv,"-scale", scaling_factor);

    /*************************************************************
     *********************   INITIAL IMAGE   *********************
     *************************************************************/

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
    cv::remap(img_bg,img_bg,map_color_x,map_color_y,cv::INTER_NEAREST);

    // Set scaling
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

    // Generate image for visualization
    cv::Mat img_contours(img_color.rows,img_color.cols,CV_8UC3,cv::Scalar(0,0,0));
    img_color.copyTo(img_contours);

    // Structuring element for dilation/erosion
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*1+1, 2*1+1 ),cv::Point( 1, 1 ) );

    /******************************************************************
     *********************   INITIAL POSE MODEL   *********************
     ******************************************************************/

    std::vector<cv::Point2d> points2D;
    points2D = getPointsForPnP(img_color_scaled, scaling_factor);
    ORK_Renderer::Renderer3d renderer = ORK_Renderer::Renderer3d(plyModel);
    Eigen::Matrix4d M = getPoseFromPnP(renderer, width, height, K_color, near, far, points2D);

//    pcl_viewer.snapPLYmodel(plyModel,M, "model");

//    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFileOBJ(objModel,mesh);
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
//    pcl::transformPointCloud(cloud, cloud, M);
//    pcl::toPCLPointCloud2(cloud, mesh.cloud);
//    pcl_viewer.cloud_viewer_.addPolygonMesh(mesh,"meshes",0);

    cv::Mat mask_out = cv::Mat(height,width,CV_8U,cv::Scalar(0));
    cv::Mat img_depth_model;
    generateModelImagesFromPose(renderer, M, img_depth_model,mask_out);
    cv::Mat mask_out_scaled;
    cv::resize(mask_out,mask_out_scaled,size);
    cv::erode(mask_out,mask_out,element);

    cv::Mat model_normals, img_model_edges;
    computeEdgesFromNormalMap(img_depth_model, model_normals,img_model_edges);
    cv::Mat img_model_edges_scaled;
    cv::resize(img_model_edges,img_model_edges_scaled,size);

    cv::Mat img_orange(img_model_edges.rows,img_model_edges.cols,CV_8UC3,cv::Scalar(0,100,255));
    img_orange.copyTo( img_contours, img_model_edges);

    imshow("edges superposed", img_contours);

    /********************************************************************
     *********************   CLOUD AND TABLE FIND   *********************
     ********************************************************************/

//    // Create cloud
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    initial_cloud_color = compute3Dpoints(img_depth,K_depth,img_color,K_color,R,t);
//    pcl::copyPointCloud(*initial_cloud_color,*initial_cloud);

//    BackGround bg(initial_cloud);
////    pcl_viewer.drawRectangle (bg.vertex2c, 255, 255, 255, 1, "table");

    /******************************************************************************
     *********************   HAND AND BACKGROUND PROCESSING   *********************
     ******************************************************************************/

    cv::Mat target_mask(height_scaled,width_scaled,CV_8U,cv::Scalar(255));

    // Hand segmentation
    cv::Mat hand_mask(height,width,CV_8U,cv::Scalar(255));
    cv::Mat non_hand_mask(height,width,CV_8U,cv::Scalar(255));
    hsSegment(img_color, 0.1, 142, 127, 0.9,0.1,hand_mask,non_hand_mask);
    cv::Mat hand_mask_scaled;
    cv::resize(hand_mask,hand_mask_scaled,size);

    // Background segmentation
    cv::Mat mask_bg(height,width,CV_8U,cv::Scalar(255));
    cv::Mat mask_fg(height,width,CV_8U,cv::Scalar(255));
    hsSegment(img_color, 0.1, img_bg, 0.3, 0.7,mask_bg, mask_fg);
    cv::Mat mask_bg_scaled;
    cv::resize(mask_bg,mask_bg_scaled,size);
    cv::Mat mask_fg_scaled;
    cv::resize(mask_fg,mask_fg_scaled,size);


    /***************************************************************
     *********************   CONFIGURE PWP3D   *********************
     ***************************************************************/

    int viewCount = 1, objectCount = 1;
    int objectId = 0, viewIdx = 0, objectIdx = 0;

    //result visualisation
    ImageUChar4* ResultImage = new ImageUChar4(width_scaled, height_scaled);
    ImageUChar4* camera = new ImageUChar4(width_scaled, height_scaled);
    cv::Mat img_color4C = cv::Mat(height_scaled,width_scaled,CV_8UC4);
    std::vector<cv::Mat> rgbChannels(4);
    cv::split(img_color_scaled, rgbChannels);
    cv::Mat alpha_mask(height_scaled,width_scaled,CV_8U,cv::Scalar(255));
    rgbChannels.push_back(alpha_mask);
    cv::merge(rgbChannels, img_color4C);
    ImageUtils::Instance()->LoadImageFromCVMat(camera, img_color4C);

    Object3D **objects = new Object3D*[objectCount];
    objects[objectIdx] = new Object3D(objectId, viewCount, (char*)objModel.c_str(), width_scaled, height_scaled);

    View3D **views = new View3D*[viewCount];
    views[viewIdx] = new View3D(viewIdx, width_scaled, height_scaled, K_color_scaled.at<double>(0,0), K_color_scaled.at<double>(1,1), K_color_scaled.at<double>(0,2), height_scaled - K_color_scaled.at<double>(1,2),width_scaled, height_scaled,near,far);

    ImageUtils::Instance()->LoadImageFromCVMat(views[viewIdx]->videoMask,
                                              target_mask);

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

//    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.01f, 0.01f, 0.01f);
    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.05f, 0.0005f, 0.0005f, 0.0005f);
//    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.01f, 0.0001f, 0.0001f, 0.0001f);
//    float step_size = 10/((K_color.at<float>(0,0)+K_color.at<float>(1,1))/2);
//    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.5f, step_size,step_size, step_size);

    /********************************************************************
     *********************   INITIALIZE PWP3D POSE  *********************
     ********************************************************************/

    VFLOAT rot[9];  

    rot[0] = M(0,0);
    rot[1] = M(0,1);
    rot[2] = M(0,2);
    rot[3] = M(1,0);
    rot[4] = M(1,1);
    rot[5] = M(1,2);
    rot[6] = M(2,0);
    rot[7] = M(2,1);
    rot[8] = M(2,2);


    objects[objectIdx]->initialPose[viewIdx]->SetFrom( M(0,3), M(1,3), M(2,3), rot);

    OptimisationEngine::Instance()->Initialise(width_scaled, height_scaled);
    OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

//    //result plot
//    VisualisationEngine::Instance()->GetImage(
//          ResultImage, GETIMAGE_PROXIMITY,
//          objects[objectIdx], views[viewIdx], objects[objectIdx]->initialPose[viewIdx]);
//    cv::Mat InitMat(height_scaled,width_scaled,CV_8UC4, ResultImage->pixels);
//    imshow("edges superposed", InitMat);


//    cv::Mat prev_img_color4C_scaled;
//    img_color4C.copyTo(prev_img_color4C_scaled);

    for (int i = img_start; i <= img_end; i++)
    {
        printf("frame %d/%d\n", i, img_end);

        // Get depth/color/edge images corrected and scaled
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

        cv::split(img_color_scaled, rgbChannels);
        cv::Mat alpha_mask(height_scaled,width_scaled,CV_8U,cv::Scalar(255));
        rgbChannels.push_back(alpha_mask);
        cv::merge(rgbChannels, img_color4C);
        ImageUtils::Instance()->LoadImageFromCVMat(camera, img_color4C);

        hsSegment(img_color, 0.1, 142, 127, 0.9,0.1,hand_mask,non_hand_mask);
        cv::resize(hand_mask,hand_mask_scaled,size);
        hsSegment(img_color, 0.1, img_bg, 0.3, 0.7,mask_bg, mask_fg);
        cv::resize(mask_bg,mask_bg_scaled,size);
        cv::resize(mask_fg,mask_fg_scaled,size);

//        cv::Mat target_mask_scaled;
//        cv::resize(target_mask,target_mask_scaled,size);
//        cv::bitwise_and(target_mask_scaled,mask_out_scaled,mask_out_scaled);
//        cv::Mat overlap_mask, overlap_mask_scaled;
//        cv::bitwise_and(mask_out_scaled,hand_mask_scaled,overlap_mask_scaled);
//        cv::resize(overlap_mask_scaled,overlap_mask,size*2);
//        overlap_mask /= 255;
//        overlap_mask = overlap_mask*(objectIdx+1);

        // Copy hands and bg to reach PWP3D
//        cv::Mat img_black(img_color.rows,img_color.cols,CV_8UC3,cv::Scalar(0,0,0));
//        img_black.copyTo(img_color,overlap_mask_scaled);
        cv::Mat img_gray(img_color_scaled.rows,img_color_scaled.cols,CV_8UC3,cv::Scalar(0,0,1));
        img_gray.copyTo(img_color_scaled,hand_mask_scaled);
//        img_black.copyTo(img_color,hand_mask);
        cv::Mat img_white(img_color_scaled.rows,img_color_scaled.cols,CV_8UC3,cv::Scalar(255,255,255));
        img_white.copyTo(img_color_scaled,mask_bg_scaled);




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

        Eigen::Matrix4d prev_M = M;

        int j= 0;
        bool loop = false;
        timer.restart();

        while (j<50)
        {
            OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);

            objects[objectIdx]->pose[viewIdx]->CopyInto(objects[objectIdx]->initialPose[viewIdx]);

            Eigen::Vector4d quat = Eigen::Vector4d(objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
                                   objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
                                   objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
                                   objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);
            Eigen::Matrix3d rot_mat = quatToMatrix(quat);
            Eigen::Vector3d trans = Eigen::Vector3d(objects[objectIdx]->pose[viewIdx]->translation->x,
                                    objects[objectIdx]->pose[viewIdx]->translation->y,
                                    objects[objectIdx]->pose[viewIdx]->translation->z);

            M.block<3,3>(0,0) = rot_mat;
            M.block<3,1>(0,3) = trans;

            Eigen::Matrix4d M_diff;
            M_diff = prev_M.inverse()*M;

            float x,y,z,roll,pitch,yaw;
            Eigen::Affine3f matrix_aux = Eigen::Translation3f(M_diff.block<3,1>(0,3).cast<float>()) * Eigen::AngleAxisf(M_diff.block<3,3>(0,0).cast<float>());
            pcl::getTranslationAndEulerAngles (matrix_aux, x, y, z, roll, pitch, yaw);

            float thresh_t = 0.001;
            float thresh_r = 0.0001;
            if ((fabs(x)<thresh_t)&&(fabs(y)<thresh_t)&&(fabs(z)<thresh_t)&&(fabs(roll)<thresh_r)&&(fabs(pitch)<thresh_r)&&(fabs(yaw)<thresh_r))
                break;
            else
            {
                if ((i==49) && (loop == false))
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

            prev_M = M;

            j++;
        }
        std::cout << timer.elapsed() << std::endl;

        objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.05f, 0.0005f, 0.0005f, 0.0005f);

//        std::cout << i << std::endl;

        generateModelImagesFromPose(renderer, M, img_depth_model,mask_out);
        cv::resize(mask_out,mask_out_scaled,size);
        computeEdgesFromNormalMap(img_depth_model, model_normals,img_model_edges);

//        pcl_viewer.cloud_viewer_.removeShape("model");
//        pcl_viewer.snapPLYmodel(plyModel,M2, "model");

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

//        img_color4C.copyTo(prev_img_color4C_scaled);
//        cv::waitKey();
    }

//    pcl_viewer.cloud_viewer_.spin();
    return 0;
}

