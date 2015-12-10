#ifndef IMAGE_CROPPER
#define IMAGE_CROPPER 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageCropper {

public:    
    ImageCropper(std::string wname);
    ImageCropper(std::string wname, int npoints);
    
    ~ImageCropper();

    void setImage(const cv::Mat &image){frame_ = image.clone(); width_= image.cols; height_= image.rows;};
    static void mouseCallbackROI(int event, int x, int y, int flags, void* param);  
    static void mouseCallbackPoints(int event, int x, int y, int flags, void* param);
    static void mouseCallbackPointsLines(int event, int x, int y, int flags, void* param);  
    
    void getROI(cv::Point2f *p1, cv::Point2f *p2){*p1 = point1_; *p2 = point2_;};
    void getPoints(std::vector<cv::Point2d>  *points){points->assign(points_.begin(),points_.end());};
    
    
private:    
    cv::Mat frame_;    
    float width_,height_;
    //points of mouse callback
    cv::Point2f point1_, point2_;
    std::string windowName_;
    
    bool drag_;
    std::vector<cv::Point2d> points_;
    int npoints_;
    void doMouseCallbackROI(int event, int x, int y, int flags);
    void doMouseCallbackPoints(int event, int x, int y, int flags);
    void doMouseCallbackPointsLines(int event, int x, int y, int flags);

};
#endif