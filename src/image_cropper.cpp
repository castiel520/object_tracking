#include <obj_recognition/image_cropper.h>

#include<stdio.h>
#include <iostream>

ImageCropper::ImageCropper(std::string windowName)
{
  windowName_  = windowName.c_str();
  cv::namedWindow(windowName_);  
  drag_ = false;
}

ImageCropper::ImageCropper(std::string windowName, int npoints)
{
  windowName_  = windowName.c_str();
  cv::namedWindow(windowName_);
  npoints_ = npoints;
  
}

ImageCropper::~ImageCropper()
{
 
}

void ImageCropper::mouseCallbackROI(int event, int x, int y, int flags, void *param)
{
    ImageCropper *self = static_cast<ImageCropper*>(param);
    self->doMouseCallbackROI(event, x, y, flags);
  
}

void ImageCropper::mouseCallbackPoints(int event, int x, int y, int flags, void *param)
{
    ImageCropper *self = static_cast<ImageCropper*>(param);
    self->doMouseCallbackPoints(event, x, y, flags);
  
}

void ImageCropper::mouseCallbackPointsLines(int event, int x, int y, int flags, void *param)
{
    ImageCropper *self = static_cast<ImageCropper*>(param);
    self->doMouseCallbackPointsLines(event, x, y, flags);
  
}

void ImageCropper::doMouseCallbackROI(int event, int x, int y, int flags)
{
    //cv::Point point1, point2;
    
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        /* left button clicked. ROI selection begins */
        point1_ = cv::Point(x, y);
        drag_  = true;
    
    }

    if (event == CV_EVENT_MOUSEMOVE)
        if (drag_)         
        {
            /* mouse dragged. ROI being selected */
            cv::Mat img1 = frame_.clone();
            point2_ = cv::Point (x, y);
            cv::rectangle(img1, point1_, point2_, CV_RGB(255, 0, 0), 3, 8, 0);
            //cv::circle(img1,point1_,34,CV_RGB(255, 0, 0), 3, 8, 0);
            cv::imshow(windowName_, img1);
            //std::cout << "the dragged point is = " << x << "," << y<< std::endl;
            cv::waitKey(50);            
        }
        
    
    if (event == CV_EVENT_LBUTTONUP)        
    {
        
        point2_ = cv::Point2f(x, y);
        point1_.x /= width_; 
        point1_.y /= height_; 
        point2_.x /= width_;    
        point2_.y /= height_;
        drag_ = false;
        //cv::imshow("CROP", img2);
        //cv::waitKey(50);
        
        //printf("ROI = [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]\n\n",point1_.x,point1_.y,point2_.x,point2_.y,height_,width_);
          
       // std::cout << "the lifted point is = " << x << "," << y<< std::endl;
    }
    if (event ==CV_EVENT_RBUTTONDOWN){      
      drag_ = false;  
      
    }
    
    
}

void ImageCropper::doMouseCallbackPoints(int event, int x, int y, int flags)
{
    //cv::Point point1, point2;
    
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        /* left button clicked. ROI selection begins */
        cv::Point2d point = cv::Point(x, y);
        std::cout << "x: " << x << " y: " << y << std::endl;
        
        if (points_.size() < npoints_)
        {
            cv::Mat img1 = frame_.clone();
               
            points_.push_back(point);
            for (int i=0; i < points_.size(); i++)
                cv::circle(img1, points_[i], 3, CV_RGB(0, 0, 255), -1, 8, 0);
            
            cv::imshow(windowName_, img1);
            cv::waitKey(50);         
//          
        }
    }

}


void ImageCropper::doMouseCallbackPointsLines(int event, int x, int y, int flags)
{
    //define colors
    std::vector<cv::Scalar> colors(5);
    colors[0] = cv::Scalar(0,0,255);
    colors[1] = cv::Scalar(255,0,0);
    colors[2] = cv::Scalar(0,255,0);
    colors[3] = cv::Scalar(255,255,0);
    colors[4] = cv::Scalar(0,255,255);
    
    
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        /* left button clicked. ROI selection begins */
        cv::Point2d point = cv::Point(x, y);
        
        
        if (points_.size() < npoints_)
        {
            cv::Mat img1 = frame_.clone();
               
            points_.push_back(point);
            for (int i=0; i < points_.size(); i++)
            {
                int color = ceil(i/4);
                cv::circle(img1, points_[i], 5, colors[color], -1, 8, 0);
                if (i%4)   //draw lines
                {
                    cv::line(img1,points_[i],points_[i-1],colors[color],3,8);
                }
            }
            
            cv::imshow(windowName_, img1);
            cv::waitKey(50);         
//          
        }
    }

}
