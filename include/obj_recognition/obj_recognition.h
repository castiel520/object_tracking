#ifndef OBJ_RECOGNITION
#define OBJ_RECOGNITION

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>


#include<Eigen/Geometry>


class ObjectRecognition {

public:
    
    ObjectRecognition();
    ObjectRecognition(const cv::Mat &K, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &T);
    ~ObjectRecognition();

    void initialize(const cv::Mat &K, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &T);
  
    void detect(const cv::Mat &color, const cv::Mat &depth,const cv::Point2f& p1,const cv::Point2f& p2);
    
    bool loadModels(const std::string& sequence_dir, const std::string& db_file);
  
    void visualize(){visualize_ = true; createVisualizer();}
    
    bool snapPLYmodel(const std::string& ply_file,const cv::Mat &T, const std::string &name);
    
    struct Model
    {
        int id; ///<model view id ... not really used anywhere but useful for debugging maybe
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; ///<model cloud from sensor point of view
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled; ///<downsampled cloud (used for first phase of pose estimate)
        Eigen::Affine3f  pose;///<pose relative to robot during recording
    };
    
    
    void getDetections(std::vector<Eigen::Matrix4f> *detections, std::vector<ObjectRecognition::Model> *models );
                   
    
    void viewPC(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& pc, const  std::string &name );

    void viewPC(const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& pc, const  std::string &name );
    
    void viewPC(const cv::Mat& color,const cv::Mat& depth, const  std::string &name );
    
    
    
    void addHandSkeleton(const cv::Mat& color,const cv::Mat& depth, 
                                                           const  std::string &name, const float &depth_limit,
                                                           const std::vector<pcl::PointXYZ>& points3D,const std::vector<int> &indices );
    
//private:
    
     
    //calibration matrices
    cv::Mat K_depth_,K_color_;
    //transformation between cameras
    cv::Mat R_,t_;

    /// stores the object type the detector is detecting
    std::string obj_type_;
    std::vector<Model> models_;
    std::string project_dir_;
    /// cluster bounds for pre-filtering (min height, min width, max height, max width) 
    std::vector<double> cluster_bounds_; 
    bool on_table_; ///< flag whether object can be on a table ... mainly for optimization right now
    bool  visualize_;
   
     double sample_size_;
    /// max distance tolerance during clustering ... needs to be larger for polaris due to large gaps when viewed from behind
    double cluster_tolerance_; 
    double score_thresh_; ///< threshold for selecting a valid match
    int min_cluster_size_;
    
    std::vector<Eigen::Matrix4f > detection_transforms_;
    std::vector<Model> model_detections_;

    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop(const cv::Mat &color, const cv::Mat &depth,
                                                const cv::Point2f& p1,const cv::Point2f& p2 );
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop(const cv::Mat &color,const cv::Point2f& p1,const cv::Point2f& p2, 
                                                const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scene);
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_ground_plane(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &scene);

    void extract_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scene, 
                                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> * clusters);
    
    void filter_clusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> * filtered_clusters);
    
    void register_pc(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);
    void getModel(int id, Model* model);
    void createVisualizer();
    void match(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& cluster, int* matchedModelID, Eigen::Matrix4f* matchedTransform, 
                     Eigen::Matrix4f* matchedTransform2);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeBG(const cv::Mat& color,const cv::Mat& depth, const  std::string &name, const float &depth_limit );

};
#endif
