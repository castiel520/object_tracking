#include <stdio.h>
#include <iomanip>
#include <string>
#include <iostream>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <obj_recognition/image_cropper.h>
//#include <obj_recognition/obj_recognition.h>
//#include <object_recognition_renderer/renderer3d.h>

//PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <GL/glut.h>

pcl::PointCloud< pcl::PointXYZ >::Ptr createPCL(const cv::Mat& depth, const cv::Mat &K_depth)
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    cv::Point3d point,pointcolor;

    cv::Mat res;
    cv::Mat Kinv = K_depth.inv();

    //std::cout << "K color  =  " << K_color_ << std::endl;
//    cv::Rect ROI(p1.x*color.cols,p1.y*color.rows,
//                        (p2.x - p1.x)*color.cols , (p2.y - p1.y)*color.rows);

    for (std::size_t i=0; i < depth.rows; i++)
    {
        for (std::size_t j=0; j < depth.cols; j++)
        {

            int u,v;
            point.x = j;
            point.y = i;
            point.z = 1;

            res = Kinv * cv::Mat( point, false);
            res.copyTo(cv::Mat ( point, false));

            if (depth.at<float>(i,j)>0.0)
            {
                point *= depth.at<float>(i,j)/1000.;

                pcl::PointXYZ p;

                p.x = point.x;
                p.y = point.y;
                p.z = point.z;

                    pc->push_back(p);

            }
        }
    }

    return pc;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr createPCL_color(const cv::Mat& color, const cv::Mat& depth,
                                                         const cv::Mat &K_depth, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &t)
{


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Point3d point,pointcolor;

    cv::Mat res;
    cv::Mat Kinv = K_depth.inv();

    //std::cout << "K color  =  " << K_color_ << std::endl;
//    cv::Rect ROI(p1.x*color.cols,p1.y*color.rows,
//                        (p2.x - p1.x)*color.cols , (p2.y - p1.y)*color.rows);

    for (std::size_t i=0; i < depth.rows; i++)
    {
        for (std::size_t j=0; j < depth.cols; j++)
        {

            int u,v;
            point.x = j;
            point.y = i;
            point.z = 1;

            res = Kinv * cv::Mat( point, false);
            res.copyTo(cv::Mat ( point, false));

            if (depth.at<float>(i,j)>0.0)
            {
                point *= depth.at<float>(i,j)/1000.;
              //  std::cout <<  point << std::endl;

                pcl::PointXYZRGB pcolor;

                res = R *  cv::Mat( point, false) +  t;
                res.copyTo(cv::Mat ( pointcolor, false));


                pcolor.x = point.x;
                pcolor.y = point.y;
                pcolor.z = point.z;



                res = K_color * cv::Mat( pointcolor,false);
                res.copyTo(cv::Mat ( pointcolor, false));
                //std::cout <<  point << std::endl;

                cv::Point2d im_point;

                im_point.x  = (int) (pointcolor.x / pointcolor.z + .5); //u
                im_point.y = (int) (pointcolor.y / pointcolor.z + .5); //v
                v = im_point.x;
                u = im_point.y;

//                if(ROI.contains(im_point))//    if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
//                {
                    uchar pr, pg, pb;

                    //Get RGB info
                    pb = color.at<cv::Vec3b>(u,v)[0];
                    pg = color.at<cv::Vec3b>(u,v)[1];
                    pr = color.at<cv::Vec3b>(u,v)[2];


                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);
                    pcColor->push_back(pcolor);

//                }
            }
        }
    }

    return pcColor;
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

void detectionToOpenGL(const Eigen::Matrix4f &M, double m[16], const cv::Mat &K, int imageWidth, int imageHeight,
      float near, float far)
{
 
 
    //OpenGL has reversed Y & Z coords
    Eigen::Matrix4f reverseYZ = Eigen::Matrix4f::Identity();   
    reverseYZ(0, 0) = 1.0;
    reverseYZ(1, 1) = -1.0;
    reverseYZ(2, 2) = -1.0;
    
    //since we are in landscape mode
    Eigen::Matrix4f rot2D = Eigen::Matrix4f::Identity();
    rot2D(0, 0) = rot2D(1, 1) = 0;
    rot2D(0, 1) =  1.0;
    rot2D(1, 0) =  -1.0;
    
    Eigen::Matrix4f projMat = Eigen::Matrix4f::Zero();
    
    projMat(0, 0) =  2*(float)K.at<double>(0, 0)/imageWidth;
    projMat(0, 2) =  -1 + (2*(float)K.at<double>(0, 2)/imageWidth);
    projMat(1, 1) =  2*(float)K.at<double>(1, 1)/imageHeight;
    projMat(1, 2) = -1 + (2*(float)K.at<double>(1, 2)/imageHeight);
    projMat(2, 2) = -(far+near)/(far-near);
    projMat(2, 3) = -2*far*near/(far-near);
    projMat(3, 2) = -1;

    
    
    
    std::cout << "OpenGL projection matrix  :" << std::endl << projMat << std::endl; 


    
    Eigen::Matrix4f mvMat = reverseYZ * M;
    projMat = rot2D * projMat;

    Eigen::Matrix4f mvp = projMat * mvMat;
    std::cout << "matrix in normal format :" << std::endl << M << std::endl; 

    std::cout << "matrix in OpenGL  format :" << std::endl << mvp << std::endl; 
    //m = &mvp(0,0);
    Eigen::Matrix4f R,mvpt;
    mvpt = mvp.transpose();
    R.block<3,3>(0,0) = mvpt.block<3,3>(0,0);
    Eigen::Vector4f t = mvp.block<4,1>(0,3);
   // std::cout << t <<std::endl;
   // R.block<4,1>(0,3) = t;
  //  std::cout << "matrix in OpenGL  format :" << std::endl << R << std::endl; 
    
     for (int i=0; i< 16; i++)
          m[i] = (double) mvp(i);
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



int readStereoCalib(std::string calibFile, cv::Mat &K_IR, cv::Mat &dist_IR,cv::Mat &K_color, cv::Mat &dist_color, cv::Mat &R, cv::Mat &t){

    cv::FileStorage fs2(calibFile, cv::FileStorage::READ);
       
    fs2["cameraMatrixLeft"] >> K_color;
    fs2["cameraMatrixRight"] >> K_IR;
    
    fs2["distCoeffsLeft"] >> dist_color;
    fs2["distCoeffsRight"] >> dist_IR;
    
    fs2["RotationMatrix"] >> R;
    fs2["Translation"] >> t;
    
    fs2.release();
    return 0;

}




int main(int argc, char **argv) {
       
    
    std::string sequence_dir;
            
    std::string project_dir = "/home/alejandro/workspace/renderer/build/";
    int img_start = 200;
    int img_end = 1000;
    
    bool save_images = false;
    bool pcl_vis = false;
    bool visualizeDepth = false ;
   
    // load left and right images
    cv::Mat img_color, img_depth;
    double near,far;
    
    sequence_dir = std::string(argv[1]) ;
    std::string scalib;
    if (std::string(argv[2]) == "-c")
        scalib = std::string(argv[3]);
    
  
    if (std::string(argv[4]) == "-s")
        img_start = atoi(argv[5]);
    if (std::string(argv[6]) == "-e")
        img_end = atoi(argv[7]);
  
   
  
    if (std::string(argv[8]) == "-n")
        near = atof(argv[9]);
  
    if (std::string(argv[10]) == "-f")
        far = atof(argv[11]);
        
        printf("near = %f, far =%f", near,far);
    //calibration matrices
    cv::Mat K_IR(3,3,CV_64F),dist_IR(3,1,CV_64F),R(3,3,CV_64F);
    cv::Mat K_color(3,3,CV_64F),dist_color(3,1,CV_64F),t(3,1,CV_64F);

    //read calibration
    readStereoCalib(scalib.c_str(),K_IR,dist_IR,K_color,dist_color,R,t);
    
    //read timestamps
    cv::Mat ts_color1,ts_depth1;
    
    ts_depth1 = readTimestamps(sequence_dir+"/depth/timestamps");
    ts_color1 = readTimestamps(sequence_dir+"/color/timestamps");
    
//    //Create visualizer
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_cleaned (new pcl::PointCloud<pcl::PointXYZRGB>);
////    viewer = createVisualizer( pc_cleaned );
    
//    cv::Mat undistortedimage_IR, undistortedimage_color;

//    std::string  src_window = "CROP";
//    ImageCropper imc(src_window);
//    cv::setMouseCallback(src_window.c_str(),&ImageCropper::mouseCallbackROI,&imc);
//    K_color/=2;
//    K_color.at<double>(2,2) = 1.0;
//    ObjectRecognition objR1(K_IR,K_color,R,t);
////    objR1.loadModels(project_dir,"db_box.txt");
//    // objR1.visualize();


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer aux"));
    viewer_->setBackgroundColor (255, 255, 255);
    viewer_->addCoordinateSystem ( 0.1 );
    viewer_->initCameraParameters();

//    pcl::visualization::PCLVisualizer viewer_("3D Viewer aux");
//    viewer_.setBackgroundColor (255, 255, 255);
//    viewer_.addCoordinateSystem ( 0.1 );
//    viewer_.initCameraParameters();
    
//    std::string plyModel = project_dir+"models/box.ply";
//    objR1.snapPLYmodel(plyModel, cv::Mat::eye(4, 4, CV_64F), "model");

//    Renderer3d renderer = Renderer3d(plyModel);
    for(int i = img_start; i <= img_end; i++)
    {
//        int i = 1;
        printf("frame %d/%d\n", i, img_end);
        
        //Depth image
        std::ostringstream ss;
        ss << std::setfill('0') << std::setw(5) << i << ".png";
        img_depth = cv::imread(sequence_dir+"/depth/"+ss.str(),CV_LOAD_IMAGE_ANYDEPTH);
        img_depth.convertTo(img_depth,CV_32F);
        
        //get the closest color image
        double timestampd1 = ts_depth1.at<double>(i);
        int imcolor1 = getTimestamp(ts_color1,timestampd1);
        
       // cv::undistort(img_depth,undistortedimage_IR,K_IR,dist_IR);

        printf("Depth image index %d, Color image index = %d\n",i,imcolor1);
        //Color image
        std::ostringstream ss_color;
        ss_color << std::setfill('0') << std::setw(5) << imcolor1 << ".jpg";
        img_color = cv::imread(sequence_dir+"/color/"+ss_color.str());
        
//        cv::Mat scaledImage;
//        cv::Size size(img_color.cols/2,img_color.rows/2);
//        cv::resize(img_color,scaledImage,size);
//        imc.setImage(scaledImage);
//        ///Image cropper
//        cv::imshow(src_window.c_str(),scaledImage);

        //cv::imshow("color",img_color);
        //cv::imshow("depth",img_depth);
        //cvWaitKey(3);
        
      
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        point_cloud_ptr = createPCL(img_depth, K_IR);



        std::cout << point_cloud_ptr->points.size() << std::endl;




        std::ostringstream ssc;
        ssc << "cloud" << i << ".png";

//        pcl::io::savePCDFileASCII ("test_pcd.pcd", *point_cloud_ptr);
//        pcl::io::loadPCDFile ("test_pcd.pcd", *point_cloud_ptr);



        //viewer_.addText3D<pcl::PointXYZ> (ss.str (), pcl::PointXYZ(0,0,2), 0.0005, 1.0, 1.0, 1.0, ssc.str ());
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cluster_handler(point_cloud_ptr);
        viewer_->addPointCloud<pcl::PointXYZ>(point_cloud_ptr, cluster_handler, ssc.str());
        viewer_->spinOnce();
        //viewer_.spin();

//        objR1.viewPC(point_cloud_ptr, "cloud");
//        objR1.viewPC(img_color, img_depth, "cloud");

//        cv::Point2f p1,p2;
//        imc.getROI(&p1,&p2);
//        objR1.detect(scaledImage,img_depth,p1,p2);
//        std::vector<Eigen::Matrix4f> detections;
//        std::vector<ObjectRecognition::Model> models;
//        objR1.getDetections(&detections,&models);
       
//        double m[16];

//        Eigen::Matrix4f M = detections[0];
        
//        int width = scaledImage.cols;
//        int height = scaledImage.rows;
        
//        double focal_length_x = K_color.at<double>(0,0);
//        double focal_length_y =  K_color.at<double>(1,1);
        

        
        
//        projectionToOpenGLTopLeft(m,K_color, width , height,near,far);
//        unsigned char *image_t = scaledImage.data;
//        renderer.set_parameters(width,height, focal_length_x, focal_length_y, near, far,m);
        
//        for (int i=0; i< 16; i++)
//            m[i] = M(i);
        
//        std::cout << "Pose from PC : \n" << M << std::endl;
        
//        cv::Mat im_out;
//        cv::Rect rect(0,0,width,height);
        
//        renderer.lookAt(m);
//        renderer.renderImageOnly(im_out,rect);
//        cv::flip(im_out,im_out,0);
//        cv::Mat mask, projected;
        
//        //cv::add(scaledImage, im_out, projected);
//        cv::add(im_out,scaledImage,  projected);
        
//        cv::imshow("3D Model", projected);
//        cvWaitKey(0);
    }
}
