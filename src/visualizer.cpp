#include "visualizer.h"
#include "custom_functions.h"

// CLASS VISUALIZER

void Viewer::drawCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double r, double g, double b, int index)
{
    char cloud_name[1024];
    sprintf (cloud_name, "cloud__%d", index);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud,r,g,b);
    bool exists;
    exists = cloud_viewer_.updatePointCloud<pcl::PointXYZ> (cloud, cloud_color, cloud_name);
    if (!exists)
        cloud_viewer_.addPointCloud<pcl::PointXYZ> (cloud, cloud_color, cloud_name);
    cloud_viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
}

void Viewer::drawColorCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int index)
{
    char cloud_name[1024];
    sprintf (cloud_name, "color_cloud__%d", index);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    bool exists;
    exists = cloud_viewer_.updatePointCloud (cloud,rgb, cloud_name);
    if (!exists)
        cloud_viewer_.addPointCloud (cloud,rgb, cloud_name);

    cloud_viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
}

void Viewer::drawSphere (pcl::PointXYZ point, double radius, double r, double g, double b, int index)
{
    char sphere_name[1024];
    sprintf (sphere_name, "sphere__%d", index);
    cloud_viewer_.removeShape(sphere_name);
    cloud_viewer_.addSphere(point,radius,r,g,b,sphere_name);
}

void Viewer::drawNormals (pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
    cloud_viewer_.removeShape("normals");
    cloud_viewer_.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.05f, "normals", 0);
}

void Viewer::drawRectangle (pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, double r, double g, double b, double opacity, std::string string)
{
    std::stringstream ss;
    ss << "rectangle_" << string;
    const std::string tmp = ss.str();
    const char* name = tmp.c_str();

    pcl::PolygonMesh polygon;

    pcl::Vertices vertex;
    for (int P = 0; P< vertices->points.size(); P++)
    {
        vertex.vertices.push_back(P);
    }

    std::vector<pcl::Vertices> vvertex;
    vvertex.push_back(vertex);

    //        sensor_msgs::PointCloud2 msg;
    //        pcl::PCLPointCloud2 msg_aux;
    //        pcl::toROSMsg( *vertices, msg );
    //        pcl_conversions::toPCL( msg, msg_aux );

    pcl::PCLPointCloud2 msg_aux;
    pcl::toPCLPointCloud2(*vertices,msg_aux);

    polygon.cloud = msg_aux;
    polygon.polygons = vvertex;

    cloud_viewer_.addPolygonMesh (polygon,name);
    //~ cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color(0),color(1),color(2), name,v2);
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, name);
    // cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.40f,name);
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,opacity,name);
    // cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,name);
}


void Viewer::drawBox(pcl::PointXYZ centroid, float z_dir, float y_dir, float x_dir, Eigen::Matrix3f dir, int Q)
{
    //~ Eigen::Vector4f vector_centroid = Vector4f(centroid.x,centroid.y,centroid.z,1);
    const Eigen::Quaternionf qfinal(dir);
    const Eigen::Vector3f tfinal = Eigen::Vector3f(centroid.x, centroid.y, centroid.z);
    char cube_name[1024];
    sprintf (cube_name, "Cube_%d", unsigned(Q));

    cloud_viewer_.addCube(tfinal, qfinal, z_dir, y_dir, x_dir,cube_name);
}

void Viewer::drawAxis (Eigen::Affine3d& pose)
{
    //~ std::cout << pose.matrix() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points_aux (new pcl::PointCloud<pcl::PointXYZ>);
    transformPointCloudCustom(*axis_points,*axis_points_aux,pose);

    *axis_points = *axis_points_aux;


    char x_arrow[1024];
    sprintf (x_arrow, "x_arrow_%d", iteration);
    char y_arrow[1024];
    sprintf (y_arrow, "y_arrow_%d", iteration);
    char z_arrow[1024];
    sprintf (z_arrow, "z_arrow_%d", iteration);

    if (iteration == 0)
    {
        cloud_viewer_.removeShape("x_arrow_0");
        cloud_viewer_.removeShape("y_arrow_0");
        cloud_viewer_.removeShape("z_arrow_0");
    }

    if ((iteration == 0) or (iteration % 2 == 0))
    {
        cloud_viewer_.addArrow (axis_points->points[1], axis_points->points[0], 1.0f, 0.0f, 0.0f, false, x_arrow);
        cloud_viewer_.addArrow (axis_points->points[2], axis_points->points[0], 0.0f, 1.0f, 0.0f, false, y_arrow);
        cloud_viewer_.addArrow (axis_points->points[3], axis_points->points[0], 0.0f, 0.0f, 1.0f, false, z_arrow);
    }
}

void Viewer::drawAxis (Eigen::Matrix3f pose)
{
    //~ std::cout << pose.matrix() << std::endl;
    Eigen::Affine3d pose_affine = createAffine3d(pose,Eigen::Vector3f(0,0,0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points_aux (new pcl::PointCloud<pcl::PointXYZ>);
    transformPointCloudCustom(*axis_points,*axis_points_aux,pose_affine);

    *axis_points = *axis_points_aux;


    char x_arrow[1024];
    sprintf (x_arrow, "x_arrow_%d", iteration);
    char y_arrow[1024];
    sprintf (y_arrow, "y_arrow_%d", iteration);
    char z_arrow[1024];
    sprintf (z_arrow, "z_arrow_%d", iteration);

    if (iteration == 0)
    {
        cloud_viewer_.removeShape("x_arrow_0");
        cloud_viewer_.removeShape("y_arrow_0");
        cloud_viewer_.removeShape("z_arrow_0");
    }

    if ((iteration == 0) or (iteration % 2 == 0))
    {
        cloud_viewer_.addArrow (axis_points->points[1], axis_points->points[0], 1.0f, 0.0f, 0.0f, false, x_arrow);
        cloud_viewer_.addArrow (axis_points->points[2], axis_points->points[0], 0.0f, 1.0f, 0.0f, false, y_arrow);
        cloud_viewer_.addArrow (axis_points->points[3], axis_points->points[0], 0.0f, 0.0f, 1.0f, false, z_arrow);
    }
}

void Viewer::createAxis ()
{
    pcl::PointXYZ pto(0,0,0);
    pcl::PointXYZ ptx(1,0,0);
    pcl::PointXYZ pty(0,1,0);
    pcl::PointXYZ ptz(0,0,1);
    axis_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    axis_points->points.push_back(pto);
    axis_points->points.push_back(ptx);
    axis_points->points.push_back(pty);
    axis_points->points.push_back(ptz);
}

void Viewer::snapPLYmodel(const std::string& plyModel, Eigen::Matrix4d T, const std::string &name)
{
//    cv::Mat T_cv;
//    cv::eigen2cv(T,T_cv);
    vtkSmartPointer<vtkTransform> vtkTrans(vtkTransform::New());
    const double vtkMat[16] = {T(0,0), T(0,1),T(0,2),T(0,3),
                               T(1,0), T(1,1),T(1,2),T(1,3),
                               T(2,0), T(2,1),T(2,2),T(2,3),
                               T(3,0), T(3,1),T(3,2),T(3,3)};
    vtkTrans->SetMatrix(vtkMat);
    cloud_viewer_.addModelFromPLYFile(plyModel,vtkTrans,  name.c_str());
    cloud_viewer_.setRepresentationToSurfaceForAllActors();
}
	
