#include <background.h>

#include<stdio.h>
#include <iostream>

#include "custom_functions.h"


BackGround::BackGround()
{
}

BackGround::BackGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    normals = getNormals(cloud,0.01);
    this->findTable(cloud,normals);
    this->computeCamera2TableMatrix();
}

BackGround::~BackGround()
{
}

void BackGround::findTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // RANSAC to find biggest plane with normal consistency
    pcl::SampleConsensusModelNormalPlane<pcl::PointXYZ, pcl::Normal>::Ptr model(new pcl::SampleConsensusModelNormalPlane<pcl::PointXYZ,pcl::Normal> (cloud_in));
    model->setInputNormals(normals);
    model->setNormalDistanceWeight(0.2);
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model);
    sac.setMaxIterations(100);
    sac.setDistanceThreshold(0.01);
    sac.computeModel ();
    std::vector<int> inliers;
    sac.getModelCoefficients(plane_vector);
    sac.getInliers (inliers);

    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    indices->indices = inliers;

    double angle = pcl::getAngle3D(plane_vector,Eigen::Vector4f(0,2,2,1)); // Pointing downwards vector
    if (pcl::rad2deg(angle) < 90)
        plane_vector = -plane_vector;

    table_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Project the model inliers to the plane
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (indices);
    proj.setInputCloud (cloud_in);
    model_coefficients.reset(new pcl::ModelCoefficients);
    for (int i = 0; i<4; i++)
        model_coefficients->values.push_back(plane_vector[i]);
    proj.setModelCoefficients (model_coefficients);
    proj.filter (*table_cloud);

    // Create a Concave Hull representation of the projected inliers
    table_cloud_concavehull.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (table_cloud);
    chull.setAlpha (0.1);
    chull.reconstruct (*table_cloud_concavehull);
}


void BackGround::computeCamera2TableMatrix ()
{
    Eigen::Matrix3f covariance;
    Eigen::Vector4f vector_centroid;
    pcl::compute3DCentroid(*table_cloud,vector_centroid);
    pcl::computeCovarianceMatrixNormalized(*table_cloud,vector_centroid,covariance);
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
    pcl::transformPointCloud(*table_cloud, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    height = max_pt.z - min_pt.z;
    length = max_pt.y - min_pt.y;
    width = max_pt.x - min_pt.x;
    float initial_area = width*length;

    // Optimize rectangle
    Eigen::Matrix3f rot(Eigen::Matrix3f::Identity());
    float angle = M_PI/180;
    rot.row(0)  = Eigen::Vector3f(cos(angle), sin(angle),0);
    rot.row(1)  = Eigen::Vector3f(-sin(angle), cos(angle),0);
    Eigen::Matrix3f  dir_rot = eigDx*rot;

    p2w.block<3,3>(0,0) = dir_rot.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
    pcl::transformPointCloud(*table_cloud_concavehull, cPoints, p2w);
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
            pcl::transformPointCloud(*table_cloud_concavehull, cPoints, p2w);
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
            pcl::transformPointCloud(*table_cloud_concavehull, cPoints, p2w);
            pcl::getMinMax3D(cPoints, min_pt, max_pt);
            current_area = (max_pt.x - min_pt.x)*(max_pt.y - min_pt.y);

            count = count+1;
        }
        eigDx = prev_eigDx;
    }

    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * vector_centroid.head<3>());
    pcl::transformPointCloud(*table_cloud_concavehull, cPoints, p2w);
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    t2c = createAffine3f(eigDx,vector_centroid.head<3>());

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    height = max_pt.z - min_pt.z;
    length = max_pt.y - min_pt.y;
    width = max_pt.x - min_pt.x;
    Eigen::Vector3f p1 = Eigen::Vector3f(-width/2, +length/2,0);
    Eigen::Vector3f p2 = Eigen::Vector3f(+width/2, +length/2,0);
    Eigen::Vector3f p3 = Eigen::Vector3f(+width/2, -length/2,0);
    Eigen::Vector3f p4 = Eigen::Vector3f(-width/2, -length/2,0);

    vertex2t.reset(new  pcl::PointCloud<pcl::PointXYZ>);
    vertex2t->push_back(getPointXYZ(p1));
    vertex2t->push_back(getPointXYZ(p2));
    vertex2t->push_back(getPointXYZ(p3));
    vertex2t->push_back(getPointXYZ(p4));
    vertex2t->push_back(pcl::PointXYZ(0,0,0));

    mean_diag = transformPoint(mean_diag,t2c);

    t2c = createAffine3f(eigDx,mean_diag);
    c2t = t2c.inverse();
    vertex2c.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*vertex2t,*vertex2c,t2c);



}
