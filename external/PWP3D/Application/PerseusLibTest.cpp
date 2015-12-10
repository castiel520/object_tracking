#include "../PerseusLib/PerseusLib.h"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>

using namespace Perseus::Utils;

void imshow(std::string name, cv::Mat image)
{
    cv::namedWindow(name,CV_WINDOW_NORMAL);
    cv::resizeWindow(name,480,270);
    cv::imshow(name, image);
    cv::waitKey(20);
//    cv::waitKey();

}

int main(void)
{
    std::string sModelPath = "/home/alejandro/Models/orange_tube_holder7.obj";
//    std::string sModelPath = "/home/alejandro/Models/mini_orange_box.obj";
    std::string sSrcImage = "/media/DATA/wetlab_v1/f2000/color/01950.jpg";
    std::string sCameraMatrix = "/home/alejandro/workspace/PWP3D/Files/CameraCalibration/f2000.cal";
    std::string sTargetMask = "/home/alejandro/Models/target_mask.png";
    std::string sHistSrc = "/media/DATA/wetlab_v1/f2000/color/01950.jpg";
    std::string sHistMask = "/home/alejandro/Models/mask.png";

//    std::string sModelPath = "/home/alejandro/workspace/PWP3D/Files/Models/Renderer/long.obj";
//    std::string sSrcImage = "/home/alejandro/workspace/PWP3D/Files/Images/Red.png";
//    std::string sCameraMatrix = "/home/alejandro/workspace/PWP3D/Files/CameraCalibration/900nc.cal";
//    std::string sTargetMask = "/home/alejandro/workspace/PWP3D/Files/Masks/480p_All_VideoMask.png";
//    std::string sHistSrc = "/home/alejandro/workspace/PWP3D/Files/Masks/Red_Source.png";
//    std::string sHistMask = "/home/alejandro/workspace/PWP3D/Files/Masks/Red_Mask.png";


  // blue car demo
  //  std::string sModelPath = "/Users/luma/Code/DataSet/Mesh/BlueCar.obj";
  //  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/Kinect.cal";
  //  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
  //  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/248-ID-3-LiveImage.png";

//  // red can demo
//  std::string sModelPath = "/Users/luma/Code/DataSet/Mesh/RedCan.obj";
//  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
//  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/Kinect.cal";
//  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
//  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
//  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/248-ID-1-LiveImage.png";

  // ---------------------------------------------------------------------------
  char str[100];
  int i;

  int width = 1920, height = 1080;
//  cv::Mat target_mask(height,width,CV_8UC1,cv::Scalar(255));
//  cv::imwrite("target_mask.png",target_mask);

  int viewCount = 1, objectCount = 1;
  int objectId = 0, viewIdx = 0, objectIdx = 0;

  Timer t;

  //result visualisation
  ImageUChar4* ResultImage = new ImageUChar4(width, height);

  // ---------------------------------------------------------------------------
  //input image
  //camera = 24 bit colour rgb
  ImageUChar4* camera = new ImageUChar4(width, height);
  ImageUtils::Instance()->LoadImageFromFile(camera, (char*)sSrcImage.c_str());

  //objects allocation + initialisation: 3d model in obj required
  Object3D **objects = new Object3D*[objectCount];

  std::cout<<"\n==[APP] Init Model =="<<std::endl;
  objects[objectIdx] = new Object3D(objectId, viewCount, (char*)sModelPath.c_str(), width, height);

  // ---------------------------------------------------------------------------
  //views allocation + initialisation: camera calibration (artoolkit format) required
  std::cout<<"\n==[APP] Init CameraMatrix =="<<std::endl;
  View3D **views = new View3D*[viewCount];
  views[viewIdx] = new View3D(0, (char*)sCameraMatrix.c_str(), width, height);


  // ---------------------------------------------------------------------------
  //histogram initialisation
  //source = 24 bit colour rgb
  //mask = 24 bit black/white png - white represents object
  //videoMask = 24 bit black/white png - white represents parts of the image that are usable
  std::cout<<"\n==[APP] Init Target ROI =="<<std::endl;
  ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
                                            (char*)sTargetMask.c_str());

  ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
                                            (char*)sHistSrc.c_str());

  ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
                                            (char*)sHistMask.c_str(), objectIdx+1);

  HistogramEngine::Instance()->UpdateVarBinHistogram(
        objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
        objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


  // ---------------------------------------------------------------------------
  //iteration configuration for one object
  IterationConfiguration *iterConfig = new IterationConfiguration();
  iterConfig->width = width; iterConfig->height = height;
  iterConfig->iterViewIds[viewIdx] = 0;
  iterConfig->iterObjectCount[viewIdx] = 1;
  iterConfig->levelSetBandSize = 8;
  iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;
  iterConfig->iterViewCount = 1;
  iterConfig->iterCount = 30;

  //step size per object and view
//  objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.2f, 0.5f, 0.5f, 10.0f);
  objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.0002f, 0.00005f, 0.00005f, 0.00005f);

//   for (int i = 0 ; i<180; i++)
//  {
  //initial pose per object and view
  // Notice the input pose here is angle, not radians for the rotation part
  VFLOAT *rot_mat;
  rot_mat =(VFLOAT [9]) {0.0786354f,0.996113f,-0.0396911f,0.486207f,-0.0730794f,-0.870782f,-0.870298f,0.0491762f,-0.490064f};
//rot_mat =(VFLOAT [9]) {0.0802093f,0.498299f,-0.863287f,0.995899f,-0.0764181f,0.0484211f,-0.0418426f,-0.863631f,-0.502385f};
    objects[objectIdx]->initialPose[viewIdx]->SetFrom(
                0.055065f, 0.112744f, 0.556275f, rot_mat);
//objects[objectIdx]->initialPose[viewIdx]->SetFrom(
//            0.0567572f, 0.088323f, 0.585495f, rot_mat);
//    Pose from PnP :
//     0.0802093   0.995899 -0.0418426  0.0567572
//      0.498299 -0.0764181  -0.863631   0.108323
//     -0.863287  0.0484211  -0.502385   0.585495
//             0          0          0          1
//    Roll: 3.04551 Pitch: -1.04175 Yaw: -1.4112
//    Roll: 174.495 Pitch: -59.6877 Yaw: -80.8558
//    Roll: -2.09766 Pitch: -0.0418548 Yaw: -1.49043
//    Roll: -120.187 Pitch: -2.3981 Yaw: -85.3954

//   for blue car demo
//    objects[objectIdx]->initialPose[viewIdx]->SetFrom( -3.0f,-4.5f,28.f, -220.90f, -207.77f, 87.48f);

//  // for red can demo
//  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
//        1.0f, 3.0f, 30.f, 180.f, 80.f, 60.f);

  //primary initilisation
  OptimisationEngine::Instance()->Initialise(width, height);

  //register camera image with main engine
  OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

  // ---------------------------------------------------------------------------
  std::cout<<"\n==[APP] Rendering object initial pose.. =="<<std::endl;
  VisualisationEngine::Instance()->GetImage(
        ResultImage, GETIMAGE_PROXIMITY,
        objects[objectIdx], views[viewIdx],
        objects[objectIdx]->initialPose[viewIdx]);

  cv::Mat InitMat(height,width,CV_8UC4, ResultImage->pixels);
  imshow("initial pose", InitMat);
//  }

  std::cout<<"[App] Finish Rendered object initial pose."<<std::endl;

//  for (i=3; i<4; i++)
//  {
//    switch (i)
//    {
//    case 0:
      iterConfig->useCUDAEF = true;
      iterConfig->useCUDARender = true;
//      break;
//    case 1:
//      iterConfig->useCUDAEF = false;
//      iterConfig->useCUDARender = true;
//      break;
//    case 2:
//      iterConfig->useCUDAEF = true;
//      iterConfig->useCUDARender = false;
//      break;
//    case 3:
//      iterConfig->useCUDAEF = false;
//      iterConfig->useCUDARender = false;
//      break;
//    }

    printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n",
           iterConfig->useCUDAEF, iterConfig->useCUDARender);

    sprintf(str, "/home/alejandro/workspace/PWP3D/Files/Results/result%04d.png", i);

    //main processing
    t.restart();
    OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
    t.check("Iteration");

    //result plot
    VisualisationEngine::Instance()->GetImage(
          ResultImage, GETIMAGE_PROXIMITY,
          objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

    //result save to file
//    ImageUtils::Instance()->SaveImageToFile(ResultImage, str);
    cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
    imshow("result", ResultMat);
//    cv::waitKey(200);

    printf("final pose result %f, %f, %f\n%f, %f, %f, %f\n\n",
           objects[objectIdx]->pose[viewIdx]->translation->x,
           objects[objectIdx]->pose[viewIdx]->translation->y,
           objects[objectIdx]->pose[viewIdx]->translation->z,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);
//  }

  //posteriors plot
  sprintf(str, "/home/alejandro/workspace/PWP3D/Files/Results/posteriors.png");
  VisualisationEngine::Instance()->GetImage(
        ResultImage, GETIMAGE_POSTERIORS,
        objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

  ImageUtils::Instance()->SaveImageToFile(ResultImage, str);

  //primary engine destructor
  OptimisationEngine::Instance()->Shutdown();

  for (i = 0; i<objectCount; i++) delete objects[i];
  delete objects;

  for (i = 0; i<viewCount; i++) delete views[i];
  delete views;

  delete ResultImage;

  cv::waitKey();
  std::cout<<"Exit pwp3D app successfully."<<std::endl;

  return 0;
}
