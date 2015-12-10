#include <obj_recognition/obj_recognition.h>

#include <vtkTransform.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
///comparator for pair of double and int
bool pairComparator( const std::pair<double,int>& l, const std::pair<double,int>& r)
{ 
    return l.first < r.first; 
}


ObjectRecognition::ObjectRecognition()
{

    cluster_bounds_.resize(4);
    min_cluster_size_ = 200;
    score_thresh_ = 0.08;
}

ObjectRecognition::ObjectRecognition(const cv::Mat &K, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &T):
    K_depth_(K),K_color_(K_color),R_(R),t_(T), visualize_(false)
{

    
    cluster_bounds_.resize(4);
    min_cluster_size_ = 200;
    score_thresh_ = 0.08;
}

ObjectRecognition::~ObjectRecognition()
{

}

void ObjectRecognition::initialize(const cv::Mat &K, const cv::Mat &K_color,const cv::Mat &R,const cv::Mat &T)
{
        K.copyTo(K_depth_);
        K_color.copyTo(K_color_); 
        R.copyTo(R_); ;
        T.copyTo(t_); 

}

void ObjectRecognition::viewPC(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& pc, const std::string &name )
{
   if(visualize_)
    {
        
        
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cluster_handler(pc);
        viewer_->addPointCloud(pc, cluster_handler, name.c_str());
    
        viewer_->spin();     
        
    }
}

void ObjectRecognition::viewPC(const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& pc, const std::string &name )
{
//   if(visualize_)
//    {


        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cluster_handler(pc);
        viewer_->addPointCloud(pc, cluster_handler, name.c_str());

        viewer_->spin();

//    }
}



void ObjectRecognition::viewPC(const cv::Mat& color, const cv::Mat& depth, const std::string& name)
{

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ground_removed (new pcl::PointCloud<pcl::PointXYZ>);
    
    point_cloud_ptr = crop(color,depth,cv::Point(0.0,0.0),cv::Point(1.0,1.0));
    //point_cloud_ptr = removeBG(color,depth,name,2.5);    
    
   // point_cloud_background_removed = remove_ground_plane(point_cloud_ptr);
    
    printf("PC has %d points\n", (int) point_cloud_ptr->points.size());
//    if(visualize_)
//    {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cluster_handler(point_cloud_ptr);
        viewer_->addPointCloud(point_cloud_ptr, cluster_handler, name.c_str());
    
        viewer_->spin();     
//    }
    
}


void ObjectRecognition::addHandSkeleton(const cv::Mat& color, const cv::Mat& depth, const std::string& name, 
                                        const float &depth_limit,const std::vector<pcl::PointXYZ>& points3D,
                                        const std::vector<int> &indices )
{
    
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
     
     
    point_cloud_ptr = removeBG(color,depth,name,depth_limit);

    std::vector<double> rgb(3);
    std::vector< std::vector <double> > colors;
    rgb[0] = 0;rgb[1] = 0;rgb[2] = 255;
    colors.push_back(rgb);
    rgb[0] = 255;rgb[1] = 0;rgb[2] = 0;
    colors.push_back(rgb);
    rgb[0] = 0;rgb[1] = 255;rgb[2] = 0;
    colors.push_back(rgb);
    rgb[0] = 255;rgb[1] = 255;rgb[2] = 0;
    colors.push_back(rgb);
    rgb[0] = 0;rgb[1] = 255;rgb[2] = 255;
    colors.push_back(rgb);
    
    for (int i=0; i < points3D.size(); i++)
    {
        int color = ceil(i/4);
        std::stringstream ss2;
        ss2 << "sphere2_" << i;
        viewer_->addSphere(points3D[i],0.0025,colors[color][0],colors[color][1],colors[color][2],ss2.str());
       
        if (i%4)
        {
            std::stringstream ss;
            ss << "line_" << i;
            
            viewer_->addLine(points3D[i],points3D[i-1],colors[color][0],colors[color][1],colors[color][2],ss.str());
            viewer_->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0,ss.str() );
        }
        
        
    }
     viewer_->spin(); 
    
    
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjectRecognition::removeBG(const cv::Mat& color, const cv::Mat& depth,
                                                                   const std::string& name, const float &depth_limit)
{

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    point_cloud_ptr = crop(color,depth,cv::Point(0.0,0.0),cv::Point(1.0,1.0));    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    
    /// Filter the points below certain height
    pass.setInputCloud (point_cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.05,depth_limit);
    pass.filter (*scene_filtered);          
      printf("PC filtered has %d points\n", (int) scene_filtered->points.size());
    //down sample point cloud
    
     //downsample cloud
    float sample_size = 0.0025;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setLeafSize(sample_size, sample_size, sample_size*3);
    vg.setInputCloud(scene_filtered);
    vg.filter(*cloud_filtered);
      printf("PC downsampled has %d points\n", (int) cloud_filtered->points.size());
    
    //remove outliers
       
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_cleaned (new pcl::PointCloud<pcl::PointXYZRGB>);
    // create the radius outlier removal filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
    
    // set input cloud
    radius_outlier_removal.setInputCloud (cloud_filtered);
    // set radius for neighbor search
    radius_outlier_removal.setRadiusSearch (0.008); //8mm
    int MinNeighborsInRadius_ = 10; //min 5 points in the sphere
    // set threshold for minimum required neighbors neighbors
    radius_outlier_removal.setMinNeighborsInRadius (MinNeighborsInRadius_);         
    // do filtering
    radius_outlier_removal.filter (*pc_cleaned);
      
    
    radius_outlier_removal.setInputCloud (pc_cleaned);
    // set radius for neighbor search
    // set threshold for minimum required neighbors neighbors
    radius_outlier_removal.setMinNeighborsInRadius (MinNeighborsInRadius_);         
    // do filtering
    radius_outlier_removal.filter (*pc_cleaned);
    
     // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pc_cleaned);
    n.setInputCloud (pc_cleaned);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*pc_cleaned, *normals, *cloud_with_normals);
    
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    //Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;

    //Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    //Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    //Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    pcl::io::saveVTKFile ("mesh.vtk", triangles);
    pcl::io::savePCDFile(name, *pc_cleaned);
    pcl::io::savePLYFile("mesh2.ply", triangles);
    
  
    printf("PC without outliers has %d points\n", (int) pc_cleaned->points.size());
    if(visualize_)
    {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cluster_handler(pc_cleaned);
        viewer_->addPointCloud(pc_cleaned, cluster_handler, name.c_str());
    
      //  viewer_->spin();     
    }
    return pc_cleaned;
    
}

bool ObjectRecognition::snapPLYmodel(const std::string& plyModel, const cv::Mat& T, const std::string &name)
{
    
    if(visualize_)
    {
        vtkSmartPointer<vtkTransform> vtkTrans(vtkTransform::New());
        const double vtkMat[16] = {T.at<double>(0,0), T.at<double>(0,1),T.at<double>(0,2),T.at<double>(0,3),
                                T.at<double>(1,0), T.at<double>(1,1),T.at<double>(1,2),T.at<double>(1,3),
                                T.at<double>(2,0), T.at<double>(2,1),T.at<double>(2,2),T.at<double>(2,3),
                                T.at<double>(3,0), T.at<double>(3,1),T.at<double>(3,2),T.at<double>(3,3)};
        vtkTrans->SetMatrix(vtkMat);
//        printf("Loading model %s\n", plyModel.c_str());
        bool error =         viewer_->addModelFromPLYFile(plyModel,vtkTrans,  name.c_str());
//        printf("Loading PLY returned %d\n", error);
        //viewer_->addText3D("New bject", pcl::PointXYZ(T.at<double>(0,3),T.at<double>(1,3),T.at<double>(2,3)+.1), 0.08, 1.0, 1.0, 1.0, "_text");    

//        viewer_->spin();
        
    }
}


bool ObjectRecognition::loadModels(const std::string& project_dir, const std::string& db_file)
{
    std::string db_path_config = project_dir+"/config/"+db_file;
    std::ifstream file;
    file.open(db_path_config.c_str());
    project_dir_ = project_dir;
    
    printf("reading file = %s",db_file.c_str());
    if(!file.is_open())
    {
        printf("File \"%s\" not found!\n", db_file.c_str());
        return false;
    }

    std::string db_path = project_dir+"/models/";
    
    
    enum LINE_DESCR {TYPE = 0, BOUNDS = 1, SAMPLESIZE = 2, CLUSTER_TOLERANCE = 3, ON_TABLE = 4, MODEL_VIEW_BEGIN};
    
    int lineNum = 0;
    while(!file.eof())
    {
        std::string line;
        getline(file, line);
        
        if(file.eof())
            break;
        
        std::stringstream lineParse(line);
        if(lineNum == TYPE) //first line is model type
        {
            lineParse >> obj_type_;
            printf("AP: loading object type %s\n", obj_type_.c_str());
        }
        else if(lineNum == BOUNDS) 
        {
            lineParse >> cluster_bounds_[0] >> cluster_bounds_[1] >> cluster_bounds_[2] >> cluster_bounds_[3];
            printf("AP: \tcluster bounds: %.2f, %.2f, %.2f, %.2f\n", cluster_bounds_[0], cluster_bounds_[1], cluster_bounds_[2], 
                        cluster_bounds_[3]);
        }
        else if(lineNum == SAMPLESIZE) //third line specifies sample size for model in meters
        {
            lineParse >> sample_size_;
            printf("AP: \tmodel sample size: %f\n", sample_size_);
        }
        else if(lineNum == CLUSTER_TOLERANCE)
        {
            lineParse >> cluster_tolerance_;
            printf("AP: \tcluster tolerance: %f\n", cluster_tolerance_);
        }
        else if(lineNum == ON_TABLE)
        {
            lineParse >> on_table_;
            printf("AP: \ton table: %d\n", on_table_);
        }
        else //all consecutive lines are model views
        {
            int id = lineNum - MODEL_VIEW_BEGIN;
            
            std::string pcd;
            float x,y,z,roll,pitch,yaw;
            lineParse >> pcd >> x >> y >> z >> roll >> pitch >> yaw;
            
            printf("AP: \tloading view %s\n", pcd.c_str());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if(pcl::io::loadPCDFile(db_path+obj_type_+"/"+pcd, *cloud) < 0)
            {
                printf("Failed loading model cloud \"%s\"!", (db_path+pcd).c_str());
                return false;
            }
            
            
            //be safe and remove NaN's and inf's (shouldn't be in this data in the first place)
            cloud->is_dense = false; //TODO: i don't trust the stored model data right now, setting is dense to false just makes sure that it checks whether the cloud has nan/inf points and removes them
            std::vector<int> idxUnusedVar;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, idxUnusedVar);
            
  
            //downsample cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(sample_size_, sample_size_, sample_size_);
            vg.setInputCloud(cloud);
            vg.filter(*cloud_filtered);
            

            //create new Model object and add to db
            Model newModel;
            newModel.cloud = cloud;
            newModel.cloud_sampled = cloud_filtered;
            newModel.id = id;
            Eigen::Vector3f t(x,y,z);
             
            newModel.pose.translation() = t;
            Eigen::Matrix3f R;
            R = Eigen::AngleAxisf(yaw*M_PI, Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(pitch*M_PI, Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(roll*M_PI, Eigen::Vector3f::UnitX());
            newModel.pose.linear() = R;
            models_.push_back(newModel);
        }
        
        ++lineNum;
    }
    printf("AP: Done loading model database");

    return true;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr ObjectRecognition::crop(const cv::Mat& color, const cv::Mat& depth,
                                                         const cv::Point2f& p1,const cv::Point2f& p2)
{

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>); 
    cv::Point3d point,pointcolor;
    
    cv::Mat res;
    cv::Mat Kinv = K_depth_.inv();
    
    //std::cout << "K color  =  " << K_color_ << std::endl;
    cv::Rect ROI(p1.x*color.cols,p1.y*color.rows, 
                        (p2.x - p1.x)*color.cols , (p2.y - p1.y)*color.rows);
    
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
    
                res = R_ *  cv::Mat( point, false) +  t_ ;
                res.copyTo(cv::Mat ( pointcolor, false));
            
                
                pcolor.x = point.x;
                pcolor.y = point.y;
                pcolor.z = point.z;
        
                
        
                res = K_color_ * cv::Mat( pointcolor,false);
                res.copyTo(cv::Mat ( pointcolor, false));      
                //std::cout <<  point << std::endl;

                cv::Point2d im_point;
                
                im_point.x  = (int) (pointcolor.x / pointcolor.z + .5); //u
                im_point.y = (int) (pointcolor.y / pointcolor.z + .5); //v
                v = im_point.x;
                u = im_point.y;
                            
                if(ROI.contains(im_point))//    if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
                {
                    uchar pr, pg, pb;

                    //Get RGB info
                    pb = color.at<cv::Vec3b>(u,v)[0];                
                    pg = color.at<cv::Vec3b>(u,v)[1];
                    pr = color.at<cv::Vec3b>(u,v)[2];
                    
                                
                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);
                    pcColor->push_back(pcolor);
                    
                }
            }
        }   
    }
 
    return pcColor;
}


pcl::PointCloud< pcl::PointXYZ >::Ptr ObjectRecognition::crop(const cv::Mat &color, const cv::Point2f& p1,const cv::Point2f& p2, 
                                                const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scene)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZ>); 
    cv::Point3d point;
    
    cv::Mat res;
    
    cv::Rect ROI(p1.x*color.cols,p1.y*color.rows, 
                        (p2.x - p1.x)*color.cols , (p2.y - p1.y)*color.rows);
    
    for (std::size_t j=0; j < scene->points.size(); j++)
    {
        point.x = scene->points[j].x;
        point.y = scene->points[j].y;
        point.z = scene->points[j].z;     
              
        
        res = K_color_ * cv::Mat( point,false);
        res.copyTo(cv::Mat ( point, false));      

        cv::Point2d im_point;            
        int u,v;
        im_point.x  = (int) (point.x / point.z + .5); //u
        im_point.y = (int) (point.y / point.z + .5); //v
        v = im_point.x;
        u = im_point.y;
                    
        if(ROI.contains(im_point))
            pcColor->push_back(scene->points[j]);       
    }   
    
    if(visualize_)
    {
     //   viewer_->removeAllPointClouds();            
        
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cluster_handler(pcColor);
        viewer_->addPointCloud(pcColor, cluster_handler, "Scene");
    
        viewer_->spin();     
        
    }
    
    return pcColor;    
}

void ObjectRecognition::extract_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scene, 
                                 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> * clusters)
{                               

      
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>) ;
     
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbpc (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbpc_sampled (new pcl::PointCloud<pcl::PointXYZ>);
      
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scenexyz (new pcl::PointCloud<pcl::PointXYZ>) ;
    scenexyz->points.resize(scene->points.size());

    for (size_t i = 0; i < scene->points.size(); i++) {
        scenexyz->points[i].x = scene->points[i].x;
        scenexyz->points[i].y = scene->points[i].y;
        scenexyz->points[i].z = scene->points[i].z;
    }
    
    std::cout << "Got point cloud  with " <<  scene->points.size() << "  points" <<std::endl;
    
    //  Downsample 
    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    
    float RadiusSearch_ = 0.002;   //2mm
    uniform_sampling.setInputCloud (scenexyz);
    uniform_sampling.setRadiusSearch (RadiusSearch_);
    uniform_sampling.compute (sampled_indices);
      
    pcl::copyPointCloud (*scenexyz, sampled_indices.points, *rgbpc_sampled);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << rgbpc_sampled->size () << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbpc_sampled_cropped (new pcl::PointCloud<pcl::PointXYZ>);     

    rgbpc_sampled_cropped = rgbpc_sampled;    
       
    /*****    Filter the point cloud    *****/
    // point cloud instance for the result
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cleaned (new pcl::PointCloud<pcl::PointXYZ>);
    // create the radius outlier removal filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
    // set input cloud
    radius_outlier_removal.setInputCloud (rgbpc_sampled_cropped);
    // set radius for neighbor search
    radius_outlier_removal.setRadiusSearch (0.008); //8mm
    int MinNeighborsInRadius_ = 5; //min 5 points in the sphere
    // set threshold for minimum required neighbors neighbors
    radius_outlier_removal.setMinNeighborsInRadius (MinNeighborsInRadius_);         
    // do filtering
    radius_outlier_removal.filter (*pc_cleaned);
    
    std::cout << "Scene sampled  points: " << rgbpc_sampled_cropped->size () << "; Removed outliers: " << pc_cleaned->size () << std::endl;

    
    
    float plane_seg_thres_ = 0.0075;
    /** get the floor and eliminate from the scene ***/    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (plane_seg_thres_);      
    seg.setMaxIterations (100);

    /// Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int plane_size_ = 5000;

    while (true)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (pc_cleaned->makeShared());
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud (pc_cleaned);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        fprintf(stderr, "plane size: %d, thresh: %d, small: %d\n", (int)cloud_p->points.size(), 
                plane_size_, (cloud_p->points.size() < plane_size_));
        if(cloud_p->points.size() < plane_size_) 
            break;
       // std::cerr << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        pc_cleaned.swap (cloud_f);
    
    }        
       
   // std::cout << "Point cloud cropped size = " << pc_cleaned->points.size() << std::endl;          
      
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    float highplane_ = -10.05;
    /// Filter the points below certain height
    pass.setInputCloud (pc_cleaned);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (highplane_, 100.0);
    pass.filter (*scene_filtered);          
      
    //std::cout << "Final point cloud without planes size = " << scene_filtered->points.size() << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered_world (new pcl::PointCloud<pcl::PointXYZ>);
    float cluster_tolerance_ = 0.01;
    scene_filtered_world = scene_filtered;
    ///extract clusters EUclidean
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(scene_filtered_world);    
    std::vector<pcl::PointIndices> clustersInd;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); //8mm
    ec.setMinClusterSize(min_cluster_size_); //need to go low for standpipe
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(scene_filtered_world);
    ec.extract(clustersInd);        
            
    for(int i = 0; i < clustersInd.size(); ++i) {      
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>); 
        pcl::PointXYZ point;
        for(int j=0; j < clustersInd[i].indices.size();j++){
            int index = clustersInd[i].indices[j];
            point.x = scene_filtered_world->points[index].x;
            point.y = scene_filtered_world->points[index].y;
            point.z = scene_filtered_world->points[index].z;      
            cluster->points.push_back(point);     
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters->push_back(cluster);
    }      
    
//     if (visualize_)
//     {
//         viewer_->removeAllPointClouds();
//             
//         printf("Number of clusters extracted: %d\n", (int) clusters->size());
//         for(int i = 0; i < clusters->size(); ++i)
//         {
//             std::stringstream ss;
//             ss << "cluster_raw_" << i;        
//             boost::shared_ptr <pcl::PointCloud <pcl::PointXYZ > > cluster;
//             pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cluster_handler(clusters->at(i));
//             viewer_->addPointCloud(clusters->at(i), cluster_handler, ss.str());
//             viewer_->spinOnce();
//         }
//         viewer_->spin();     
//     }
    
    
   
}

void ObjectRecognition::filter_clusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, 
                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* filtered_clusters)
{
    
     // filter the clusters that do not fit the object boundaries
    printf("Clusters detected = %d \n", (int) clusters.size());
    
    for(int i = 0; i < clusters.size(); ++i)    
    {
      
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = clusters[i];
        
        
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);
        
        float height = maxPt.y-minPt.y;
        float width = std::max(maxPt.x-minPt.x, maxPt.z-minPt.z);
        //if(height <= OBJ_MAX_HEIGHT && height >= OBJ_MIN_HEIGHT &&  width <= OBJ_MAX_WIDTH && width >= OBJ_MIN_WIDTH) //(min height, min width, max height, max width) 
        printf("width = %lf,  height =%lf\n",width,height);
    
        // cluster needs to satisfy min and max size constraints
        if(width >= cluster_bounds_[0] && width <= cluster_bounds_[1] && height >= cluster_bounds_[2] && height <= cluster_bounds_[3]
            && cluster->size() >= min_cluster_size_) 
        {
            filtered_clusters->push_back(cluster);
            printf("cluster candidate added (h:%f,w:%f)\n", height, width);
        }
        
    }
    
//     if (visualize_)
//     {
//         viewer_->removeAllPointClouds();
//             
//         printf("Number of clusters extracted: %d\n", (int) filtered_clusters->size());
//         for(int i = 0; i < filtered_clusters->size(); ++i)
//         {
//             std::stringstream ss;
//             ss << "cluster_cadidate_" << i;        
//             pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cluster_handler(filtered_clusters->at(i));
// 
//             viewer_->addPointCloud(filtered_clusters->at(i), cluster_handler, ss.str());
//             viewer_->spinOnce();
//         }
//         viewer_->spin();     
//     }

}





void ObjectRecognition::detect(const cv::Mat& color, const cv::Mat& depth,
                       const cv::Point2f& p1,const cv::Point2f& p2)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_world (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ground_removed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_cropped (new pcl::PointCloud<pcl::PointXYZ>);
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters,filtered_clusters;         
    point_cloud_ptr = crop(color,depth,cv::Point(0.0,0.0),cv::Point(1.0,1.0));    
    point_cloud_ground_removed = remove_ground_plane(point_cloud_ptr);
    
    point_cloud_cropped = crop(color,p1,p2,point_cloud_ground_removed);    
    
    Eigen::Matrix3f CtoW;
    //point cloud in world coordinates
    float yaw, pitch, roll;
    yaw =  M_PI/2;
    pitch = M_PI;
    roll  = M_PI/2;
    CtoW = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
                
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = CtoW;
                
    pcl::transformPointCloud(*point_cloud_cropped, *point_cloud_world,T);           
    extract_clusters(point_cloud_world,&clusters);
    filter_clusters(clusters, &filtered_clusters);
    register_pc(filtered_clusters);
}

pcl::PointCloud< pcl::PointXYZ >::Ptr ObjectRecognition::remove_ground_plane(const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& scene)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cleaned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>) ;
    
    pc_cleaned->points.resize(scene->points.size());

    for (size_t i = 0; i < scene->points.size(); i++) {
        pc_cleaned->points[i].x = scene->points[i].x;
        pc_cleaned->points[i].y = scene->points[i].y;
        pc_cleaned->points[i].z = scene->points[i].z;
    }
    
    float plane_seg_thres_ = 0.005;
    /** get the floor and eliminate from the scene ***/    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (plane_seg_thres_);      
    seg.setMaxIterations (100);

    /// Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int plane_size_ = 30000;

    while (true)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (pc_cleaned->makeShared());
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud (pc_cleaned);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        fprintf(stderr, "plane size: %d, thresh: %d, small: %d\n", (int)cloud_p->points.size(), 
                plane_size_, (cloud_p->points.size() < plane_size_));
        if(cloud_p->points.size() < plane_size_) 
            break;
       // std::cerr << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        pc_cleaned.swap (cloud_f);
    
    }        
    return pc_cleaned;
}

//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
void ObjectRecognition::createVisualizer ()
{
    
    viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_->setBackgroundColor (0, 0, 0);
    viewer_->addCoordinateSystem ( 0.1 );
    viewer_->initCameraParameters();
    
}

void ObjectRecognition::getModel(int id, Model* model)
{
    if(id>=0 && id < models_.size())
    {
        *model = models_[id];
    }
    else
    {
        *model = Model();
        printf("AP: no model with id %d! (%d models in db)\n", id, (int) models_.size());
    }
}

void ObjectRecognition::match(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& cluster, int* matchedModelID, 
                              Eigen::Matrix4f* matchedTransform, Eigen::Matrix4f* matchedTransform2)
{

    std::vector<std::pair<double,int> > model_scores(models_.size()); //store model score with index for sorting
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > model_transforms(models_.size());

    //downsample cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(sample_size_, sample_size_, sample_size_);
    vg.setInputCloud(cluster);
    vg.filter(*cluster_sampled);
    printf("cluster has %d points after sampling (%d before)\n", (int) cluster_sampled->size(), (int) cluster->size());

    int numThreads = models_.size(); 
    int schedule_chunk_size = 1;
         
  
            
    
    #pragma omp parallel for num_threads(numThreads) schedule(dynamic, schedule_chunk_size)
    for(int modelIdx = 0; modelIdx < models_.size(); ++modelIdx)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr model = models_[modelIdx].cloud_sampled;

        Eigen::Vector4f centModel, centCluster;
        pcl::compute3DCentroid(*model, centModel);
        pcl::compute3DCentroid(*cluster_sampled, centCluster);
       
        /// Model transformed into the cluster reference system
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        
        Eigen::Matrix4f T_cluster = Eigen::Matrix4f::Identity(); 
         T_cluster.block<3,1>(0,3) = centCluster.head(3) - centModel.head(3);
        

      
        pcl::transformPointCloud(*model, *model_transformed, T_cluster);   
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelRegistered(new pcl::PointCloud<pcl::PointXYZ> );
        
        pcl::ScopeTime t("ICP...");

        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ >::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
          
        registration->setInputSource(model_transformed);
        registration->setInputTarget(cluster_sampled);
        registration->setMaxCorrespondenceDistance(0.25);
        registration->setRANSACOutlierRejectionThreshold(0.1);
        registration->setTransformationEpsilon(0.000001);
        registration->setMaximumIterations(50);
        
        printf("Input to ICP: inputSource: %dpts, inputTarget: %dpts\n", (int) model_transformed->size(),(int) cluster_sampled->size());

        registration->align(*modelRegistered);
        Eigen::Matrix4f transformation_matrix = registration->getFinalTransformation();
        //printf("score: %f\n", registration->getFitnessScore());
        model_scores[modelIdx] = std::make_pair(registration->getFitnessScore(), modelIdx);
        model_transforms[modelIdx] = transformation_matrix *  T_cluster;
        
    }

    //sort model scores (lower score = better)
    std::sort(model_scores.begin(), model_scores.end(), pairComparator);
        
    if(model_scores[0].first < score_thresh_)
    {
        *matchedModelID = model_scores[0].second;
        
        //calculate final transformation
        Eigen::Affine3f initPose  = models_[*matchedModelID].pose;
        std::cout << "Init model pose \n" << initPose.matrix() << endl;
        
        *matchedTransform =  model_transforms[*matchedModelID] * initPose.matrix();
        *matchedTransform2 =  model_transforms[*matchedModelID];
        
        printf("AP: Best match: view %d with score of %f\n", model_scores[0].second, model_scores[0].first);
        std::cout << *matchedTransform << std::endl;;
    }
    else
    {
        printf("AP: No matches found for cluster (might be a good thing!)\n");
    }
}

void ObjectRecognition::register_pc(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_candidates)
{

    printf("AP: starting detection ...\n");
    
    if (visualize_)
    {
        //viewer_->addPointCloud(scene_, scene_handler_, "scene");
    }
    

    pcl::ScopeTime t_all;

    #pragma omp parallel for 
    for(int i = 0; i < cluster_candidates.size(); ++i)
    {
        printf("AP: running detection on cluster %d\n", i);
        
        Eigen::Matrix4f detection_transform, detection_transform2;
        Eigen::Matrix3f R;
        Eigen::Vector3f tras;
        Eigen::Matrix4f TtoOrigin = Eigen::Matrix4f::Identity(); 
        int detection_id = -1;
        pcl::ScopeTime t;
        match(cluster_candidates[i], &detection_id, &detection_transform, &detection_transform2);
        
//         std::cout <<"detected transform \n" << detection_transform << std::endl;
//         std::cout << "Back to origin \n" << TtoOrigin << std::endl;
//         
        
        detection_transform = TtoOrigin * detection_transform;
        detection_transform2 = TtoOrigin * detection_transform2;
        
//         std::cout <<"after applying \n" << detection_transform << std::endl;
        
        printf("Detection for cluster %d took %fs\n", i, t.getTimeSeconds());       
        
        

        if(detection_id != -1)
        {
            Model m;
            getModel(detection_id, &m);
            model_detections_.push_back(m);
            
            
            
            
            
             Eigen::Matrix3f CtoW;
            Eigen::Matrix4f CtoW4  = Eigen::Matrix4f::Identity(); 
            //point cloud in world coordinates
            float yaw, pitch, roll;
            yaw = M_PI/2;
            pitch =  M_PI;
            roll  = M_PI/2;
            CtoW = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            
            
            CtoW4.block<3,3>(0,0) = CtoW.transpose();
            detection_transform = CtoW4 * detection_transform ;
            detection_transform2 = CtoW4 *  detection_transform2 ;

            //detection_transform = detection_transform.inverse().eval();
            //detection_transform2 = detection_transform2.inverse().eval();
            
            detection_transforms_.push_back(detection_transform);
        
            if(visualize_)   
            {
                vtkSmartPointer<vtkTransform> vtkTrans(vtkTransform::New());
                const double vtkMat[16] = {(detection_transform)(0,0), (detection_transform)(0,1),(detection_transform)(0,2),(detection_transform)(0,3),
                                        (detection_transform)(1,0), (detection_transform)(1,1),(detection_transform)(1,2),(detection_transform)(1,3),
                                        (detection_transform)(2,0), (detection_transform)(2,1),(detection_transform)(2,2),(detection_transform)(2,3),
                                        (detection_transform)(3,0), (detection_transform)(3,1),(detection_transform)(3,2),(detection_transform)(3,3)};
                vtkTrans->SetMatrix(vtkMat);
                std::stringstream visMatchName;
                static int visMatchedModelCounter = 0;
                visMatchName << obj_type_ << visMatchedModelCounter++;
                printf("loading  %s for cluster %d\n", obj_type_.c_str(),i);
                std::string plyModel = project_dir_+"/models/"+obj_type_+"/"+obj_type_+".ply";
                viewer_->addModelFromPLYFile(plyModel, vtkTrans, visMatchName.str());
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudMatched(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(*m.cloud, *modelCloudMatched, detection_transform2);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_cloud_handler(modelCloudMatched, 0, 255, 0);
                viewer_->addPointCloud(modelCloudMatched, match_cloud_handler, visMatchName.str()+"_cloud");
                viewer_->addText3D(visMatchName.str(), pcl::PointXYZ(detection_transform(0,3),detection_transform(1,3),detection_transform(2,3)+.1), 0.08, 1.0, 1.0, 1.0, visMatchName.str()+"_text");    
                viewer_->spinOnce();
            }
                
        }
        printf("cluster %d done\n", i);
    }
    
    
    if (visualize_)
    {
        viewer_->spin();
    }
    printf("Detection for all clusters took %fs\n", t_all.getTimeSeconds());
}



void ObjectRecognition::getDetections(std::vector<Eigen::Matrix4f> *detections, std::vector<ObjectRecognition::Model> *models )
{
    detections->assign(detection_transforms_.begin(),detection_transforms_.end() );
    models->assign(model_detections_.begin(),model_detections_.end());
    
}

