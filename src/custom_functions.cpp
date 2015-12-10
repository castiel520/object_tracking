#include "custom_functions.h"

//~ template <typename PointT> 
void transformPointCloudCustom(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out, const Eigen::Affine3d &transform)
{
	int count = 0;
	
	if (&cloud_in != &cloud_out)
	{
		// Note: could be replaced by cloud_out = cloud_in
		cloud_out.header   = cloud_in.header;
		cloud_out.is_dense = cloud_in.is_dense;
		cloud_out.width    = cloud_in.width;
		cloud_out.height   = cloud_in.height;
		cloud_out.points.reserve (cloud_out.points.size ());
		cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
		cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
		cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
	}

	//~ cloud_out.points.clear(); //comprobar si está bien quitado
	
	for (size_t i = 0; i < cloud_in.points.size (); i++)
	{
		
		if (pcl::isFinite (cloud_in.points[i])//.x) and !pcl::isFinite (cloud_in.points[i].y) and !pcl::isFinite (cloud_in.points[i].z)
		//~ and !isnan(cloud_in.points[i].x) and !isnan (cloud_in.points[i].y) and !isnan (cloud_in.points[i].z)
		)
		{
			cloud_out.points.push_back(cloud_in.points[i]);
			Eigen::Vector3f pt = Eigen::Vector3f(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);

			cloud_out.points[count].x = static_cast<float> (transform (0, 0) * pt (0) + transform (0, 1) * pt (1) + transform (0, 2) * pt (2) + transform (0, 3));
			cloud_out.points[count].y = static_cast<float> (transform (1, 0) * pt (0) + transform (1, 1) * pt (1) + transform (1, 2) * pt (2) + transform (1, 3));
			cloud_out.points[count].z = static_cast<float> (transform (2, 0) * pt (0) + transform (2, 1) * pt (1) + transform (2, 2) * pt (2) + transform (2, 3));
			
			count++;
		}
    }
    
    cloud_out.height = 1;
    cloud_out.width = cloud_out.points.size();

}


//~ template <typename PointT> 
void transformPointCloudCustom(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, const Eigen::Affine3d &transform)
{
	int count = 0;
	
	if (&cloud_in != &cloud_out)
	{
		// Note: could be replaced by cloud_out = cloud_in
		cloud_out.header   = cloud_in.header;
		cloud_out.is_dense = cloud_in.is_dense;
		cloud_out.width    = cloud_in.width;
		cloud_out.height   = cloud_in.height;
		cloud_out.points.reserve (cloud_out.points.size ());
		cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
		cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
		cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
	}

	//~ cloud_out.points.clear(); //comprobar si está bien quitado
	
	for (size_t i = 0; i < cloud_in.points.size (); i++)
	{
		//~ std::cout << "hola" << std::endl;
		if (pcl::isFinite (cloud_in.points[i])//.x) and !pcl::isFinite (cloud_in.points[i].y) and !pcl::isFinite (cloud_in.points[i].z)
		//~ and !isnan(cloud_in.points[i].x) and !isnan (cloud_in.points[i].y) and !isnan (cloud_in.points[i].z)
		)
		{
			cloud_out.points.push_back(cloud_in.points[i]);
			Eigen::Vector3f pt = Eigen::Vector3f(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);

			cloud_out.points[count].x = static_cast<float> (transform (0, 0) * pt (0) + transform (0, 1) * pt (1) + transform (0, 2) * pt (2) + transform (0, 3));
			cloud_out.points[count].y = static_cast<float> (transform (1, 0) * pt (0) + transform (1, 1) * pt (1) + transform (1, 2) * pt (2) + transform (1, 3));
			cloud_out.points[count].z = static_cast<float> (transform (2, 0) * pt (0) + transform (2, 1) * pt (1) + transform (2, 2) * pt (2) + transform (2, 3));
			
			count++;
		}
    }
    
    cloud_out.height = 1;
    cloud_out.width = cloud_out.points.size();

}

float getAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB)
{
	vectorA.normalize();
	vectorB.normalize();
	float dot = vectorA.dot(vectorB);
	float theta;
	theta = acosf(dot);
	
	return theta;
}

float getAngle(Eigen::Vector4f vectorA, Eigen::Vector4f vectorB)
{
	Eigen::Vector3f vectorAaux, vectorBaux;
	vectorAaux = vectorA.head(3);
	vectorBaux = vectorB.head(3);
	
	vectorAaux.normalize();
	vectorBaux.normalize();
	float dot = vectorAaux.dot(vectorBaux);
	float theta;
	theta = acosf(dot);
	
	return theta;
}

float getNegativeAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB)
{
	vectorA.normalize();
	vectorB.normalize();
	float dot = vectorA.dot(vectorB);
	float theta;
	theta = acosf(dot);

	if (theta > M_PI)
		return theta-2*M_PI;
	else
		return theta;
}

float getNegativeHorizontalAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB)
{

	Eigen::VectorXf vectorXZA(2);
	Eigen::VectorXf vectorXZB(2);

	vectorXZA(0) = vectorA(0);
	vectorXZA(1) = vectorA(2);

	vectorXZB(0) = vectorB(0);
	vectorXZB(1) = vectorB(2);

	vectorXZA.normalize();
	vectorXZB.normalize();


	float dot = vectorXZA.dot(vectorXZB);
	float theta, theta_aux;
	theta_aux = acosf(dot);

	if (theta_aux > M_PI)
		theta = theta_aux-2*M_PI;
	else
		theta = theta_aux;

	return theta;

}

float getHorizontalAngle(Eigen::Vector3f vectorA, Eigen::Vector3f vectorB)
{

	Eigen::VectorXf vectorXZA(2);
	Eigen::VectorXf vectorXZB(2);

	vectorXZA(0) = vectorA(0);
	vectorXZA(1) = vectorA(2);

	vectorXZB(0) = vectorB(0);
	vectorXZB(1) = vectorB(2);

	vectorXZA.normalize();
	vectorXZB.normalize();


	float dot = vectorXZA.dot(vectorXZB);
	float theta;
	theta = acosf(dot);

	return theta;
}

//bool sortByHeight(const Plane &lstep, const Plane &rstep)
//{
//	return fabs(lstep.centroid2f.y) < fabs(rstep.centroid2f.y); // <
//}

//bool sortPlanesBySize(const Plane &lhs, const Plane &rhs)
//{
//    return lhs.cloud->points.size() > rhs.cloud->points.size(); //> los ordena de mayor a menor, < al revés
//}

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud)
{
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<float> pointRadiusSquaredDistance;
	std::vector<int> pointIdxRadiusSearch;
	int neighbours;
	neighbours = kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);
	
	pcl::copyPointCloud(*cloud,pointIdxRadiusSearch,*neighbouring_cloud);
	
	return neighbours;
}

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius)
{
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<float> pointRadiusSquaredDistance;
	std::vector<int> pointIdxRadiusSearch;
	int neighbours;
	neighbours = kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);

	return neighbours;
}


pcl::PointXYZ getPointXYZ(Eigen::Vector3f vector)
{
	pcl::PointXYZ point(vector(0),vector(1),vector(2));
	
	return point;
}


//Plane getBiggestStep (std::vector<Plane> vPlanes)
//{
//	int index = 0;
//	int size = 0;

//	for (int Q = 0; Q<vPlanes.size(); Q++)
//	{
		
//		if (Q == 0)
//		{
//			size = vPlanes[Q].cloud->points.size();
//		}
//		else
//		{
//			// if (vPlanes[Q].level > 0)
//			if (vPlanes[Q].isValid)
//			{
//				int current_size = vPlanes[Q].cloud->points.size();
//				if (current_size > size)
//				{
//					index = Q;
//					size = current_size;
//				}
//			}
//		}
	
		
//	}
//	return vPlanes[index];
//}

//Plane getBestStep (GlobalScene gscene, std::vector<Plane> vPlanes)
//{
//	int index = 0;
//	float max_extent = 0;

//	float sum_of_contour_areas = 0;

//// std::cout << "Sum of contour areas" << std::endl;
//	for (int Q = 1; Q<vPlanes.size(); Q++)
//	{
//		// std::cout << "Q = " << Q << std::endl;
//		// if (vPlanes[Q].contour_area == 0)
//			sum_of_contour_areas += vPlanes[Q].getContourArea();
//		// else
//		// 	sum_of_contour_areas += vPlanes[Q].contour_area;
//			// std::cout << "suman " << sum_of_contour_areas << std::endl;
//	}

//	// std::cout << "Sum of contour areas: " << sum_of_contour_areas << std::endl;
//	// std::cout << "Planos a comprobar: " << std::endl;
//	for (int Q = 1; Q<vPlanes.size(); Q++)
//	{
//		// std::cout << "Q = " << Q << std::endl;
//		if (vPlanes[Q].eigDx.isZero(0))
//			vPlanes[Q].getPrincipalDirections();


//		vPlanes[Q].getStairDirFromPCA(gscene.c2f);
		
//		// std::cout << "Sum of rectangle areas" << std::endl;

//		float sum_of_rectangle_areas = 0;
//		for (int P = 1; P<vPlanes.size(); P++)
//		{
//			// std::cout << "P = " << P << std::endl;
//			sum_of_rectangle_areas += vPlanes[P].getRectangleArea(vPlanes[Q].eigDx);
//			// std::cout << "suman " << sum_of_rectangle_areas << std::endl;
//		}

//		// std::cout << "Sum of rectangle areas: " << sum_of_rectangle_areas << std::endl;

//		float extent = sum_of_contour_areas/sum_of_rectangle_areas;
//		// std::cout << "EXTENT: " << extent << std::endl;
//		if (extent > max_extent)
//		{
//			max_extent = extent;
//			index = Q;
//		}

//		// std::cout << "Best step is " << index << std::endl;
//	}
//	return vPlanes[index];
//}

Eigen::Affine3d createAffine3d(Eigen::Matrix3d rotation, Eigen::Vector3d translation)
{
	return Eigen::Translation3d(translation) * Eigen::AngleAxisd(rotation);
}

Eigen::Affine3d createAffine3d(Eigen::Matrix3f rotation, Eigen::Vector3f translation)
{
	return Eigen::Translation3d(translation.cast<double>()) * Eigen::AngleAxisd(rotation.cast<double>());
}

Eigen::Affine3f createAffine3f(Eigen::Matrix3f rotation, Eigen::Vector3f translation)
{
    return Eigen::Translation3f(translation) * Eigen::AngleAxisf(rotation);
}

void sayAffine(Eigen::Affine3d matrix)
{
    float x,y,z,roll,pitch,yaw;
    Eigen::Affine3f matrix_aux = matrix.cast<float>();
    pcl::getTranslationAndEulerAngles (matrix_aux, x, y, z, roll, pitch, yaw);

	std::cout << "X: " << x << std::endl;
	std::cout << "Y: " << y << std::endl;
	std::cout << "Z: " << z << std::endl;

	std::cout << "Roll: " << roll*180/M_PI << std::endl;
	std::cout << "Pitch: " << pitch*180/M_PI << std::endl;
	std::cout << "Yaw: " << yaw*180/M_PI << std::endl;
}

void sayAffine(Eigen::Affine3f matrix)
{
    float x,y,z,roll,pitch,yaw;
    pcl::getTranslationAndEulerAngles (matrix, x, y, z, roll, pitch, yaw);

    std::cout << "X: " << x << std::endl;
    std::cout << "Y: " << y << std::endl;
    std::cout << "Z: " << z << std::endl;

    std::cout << "Roll: " << roll*180/M_PI << std::endl;
    std::cout << "Pitch: " << pitch*180/M_PI << std::endl;
    std::cout << "Yaw: " << yaw*180/M_PI << std::endl;
}

Eigen::Vector3f transformPoint (Eigen::Vector3f point, Eigen::Affine3f T)
  {
    Eigen::Vector4f point_aux (point(0),point(1),point(2),1);

    Eigen::Vector4f point_aux_2 = T * point_aux;

    return point_aux_2.head<3>();
  }

std::vector<Eigen::Vector3f> transformPoint (std::vector<Eigen::Vector3f> point_in, Eigen::Affine3f T)
  {
    std::vector<Eigen::Vector3f> point_out;

    for (int i = 0 ; i < point_in.size(); i++)
    {

        Eigen::Vector4f point_aux (point_in[i](0),point_in[i](1),point_in[i](2),1);

        Eigen::Vector4f point_aux_2 = T * point_aux;
        point_out.push_back(point_aux_2.head<3>());

    }

    return point_out;
  }

cv::Mat falseColor(cv::Mat img_depth){
        double min;
        double max;
        cv::Mat falseColorsMap;
        cv::minMaxIdx(img_depth, &min, &max);
//        std::cout << min << " " << max << std::endl;
        cv::Mat adjMap;
        img_depth.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
//        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);
//        return falseColorsMap;
        return adjMap;
}

Eigen::Matrix3d quatToMatrix(Eigen::Vector4d q)
{

    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();

    double sqw = q[3]*q[3];
    double sqx = q[0]*q[0];
    double sqy = q[1]*q[1];
    double sqz = q[2]*q[2];

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    mat(0,0) = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    mat(1,1) = (-sqx + sqy - sqz + sqw)*invs ;
    mat(2,2) = (-sqx - sqy + sqz + sqw)*invs ;

    double tmp1 = q[0]*q[1];
    double tmp2 = q[2]*q[3];
    mat(1,0) = 2.0 * (tmp1 + tmp2)*invs ;
    mat(0,1) = 2.0 * (tmp1 - tmp2)*invs ;

    tmp1 = q[0]*q[2];
    tmp2 = q[1]*q[3];
    mat(2,0) = 2.0 * (tmp1 - tmp2)*invs ;
    mat(0,2) = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q[1]*q[2];
    tmp2 = q[0]*q[3];
    mat(2,1) = 2.0 * (tmp1 + tmp2)*invs ;
    mat(1,2) = 2.0 * (tmp1 - tmp2)*invs ;

    return mat;
}

void imshow(std::string name, cv::Mat image)
{
    cv::namedWindow(name,CV_WINDOW_NORMAL);
    cv::resizeWindow(name,480,270);
    cv::imshow(name, image);
    cv::waitKey(20);
}

cv::Mat canny(cv::Mat img, float th1 = 0.2, float th2 = 20)
{
    cv::Mat img_gray, edges;
    if (img.channels()>1)
        cv::cvtColor(img,img_gray,CV_RGB2GRAY);
    else
        img.convertTo(img_gray,CV_8UC1);
//    double otsu_threshold; //http://stackoverflow.com/questions/4292249/automatic-calculation-of-low-and-high-thresholds-for-the-canny-operation-in-open
//    otsu_threshold = cv::threshold(img_gray,dst,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU);
//    double high_thresh_val  = otsu_threshold*2,
//           lower_thresh_val = otsu_threshold * 0.5;

    double high_thresh_val  = th1,
           lower_thresh_val = th2;
    cv::Canny(img_gray,edges, lower_thresh_val,high_thresh_val);

    return edges;
}

cv::Mat edges_from_normals(cv::Mat img, cv::Mat depth, float thresh)
{
    cv::Mat edges(img.rows,img.cols,CV_8UC1,cv::Scalar(0));
    for (std::size_t i=0; i < img.rows-1; i++)
    {
        for (std::size_t j=0; j < img.cols-1; j++)
        {
            if (depth.at<float>(i,j) > 0)
            {
                Eigen::Vector3f v = Eigen::Vector3f(((float)img.at<cv::Vec3b>(i,j)[0])/255*2-1, ((float)img.at<cv::Vec3b>(i,j)[1])/255*2-1,((float) img.at<cv::Vec3b>(i,j)[2])/255*2-1);
                Eigen::Vector3f vh = Eigen::Vector3f(((float)img.at<cv::Vec3b>(i,j+1)[0])/255*2-1,((float) img.at<cv::Vec3b>(i,j+1)[1])/255*2-1, ((float)img.at<cv::Vec3b>(i,j+1)[2])/255*2-1);
                Eigen::Vector3f vv = Eigen::Vector3f(((float)img.at<cv::Vec3b>(i+1,j)[0])/255*2-1,((float) img.at<cv::Vec3b>(i+1,j)[1])/255*2-1, ((float)img.at<cv::Vec3b>(i+1,j)[2])/255*2-1);

//                std::cout << v << std::endl << std::endl;
//                std::cout << vh << std::endl << std::endl;
//                std::cout << vv << std::endl << std::endl << std::endl;
//                std::cout << getAngle(v,vh) << " " << getAngle(v,vh)*180/M_PI << std::endl << std::endl;
//                std::cout << getAngle(v,vv) << " " << getAngle(v,vv)*180/M_PI << std::endl << std::endl;
                if (fabs(getNegativeAngle(v,vh)) > thresh*M_PI/180 || fabs(getNegativeAngle(v,vv)) > thresh*M_PI/180)
                {
//                    std::cout << "entro aqui" << std::endl;
                    edges.at<uchar>(i,j) = 255;
                }
            }
        }
    }
    return edges;
}

cv::Mat canny_three_channels(cv::Mat img, float th1 = 0.2, float th2 = 20)
{

    double high_thresh_val  = th1,
           lower_thresh_val = th2;

    cv::Mat ch1, ch2, ch3;
    // "channels" is a vector of 3 Mat arrays:
    std::vector<cv::Mat> channels(3);

    // split img:
    cv::split(img, channels);
    // get the channels (dont forget they follow BGR order in OpenCV)
    ch1 = channels[0];
    ch2 = channels[1];
    ch3 = channels[2];

    cv::Mat grad(img.rows,img.cols,CV_16S,cv::Scalar(0));
    cv::Mat img_gray(img.rows,img.cols,CV_16S,cv::Scalar(0));
    for (int i = 0; i < 3; i++)
    {
        img_gray = channels[i];
//        imshow("imggray", img_gray);

//        int scale = 1;
//        int delta = 0;
//        /// Generate grad_x and grad_y
//        cv::Mat grad_x, grad_y;
//        cv::Mat abs_grad_x, abs_grad_y;

//        /// Gradient X
//        //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
//        cv::Sobel( img_gray, grad_x, CV_16S, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
//        cv::convertScaleAbs( grad_x, abs_grad_x );

//        /// Gradient Y
//        //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
//        cv::Sobel( img_gray, grad_y, CV_16S, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
//        cv::convertScaleAbs( grad_y, abs_grad_y );

//        /// Total Gradient (approximate)
    cv::Mat temp_grad(img.rows,img.cols,CV_16S,cv::Scalar(0));
//        cv::add(grad_x,grad_y,temp_grad);
//        cv::add(temp_grad,grad,grad);
        //        grad = grad + grad_x + grad_y;
        //        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, temp_grad );
        //        cv::convertScaleAbs( temp_grad, temp_grad );
        //        cv::convertScaleAbs( grad, grad );
        //        imshow("temp_grad",temp_grad);
        //        imshow("grad",grad);
//                cv::waitKey();
//    std::cout << img_gray.rows << " " << temp_grad.rows << " " << grad.rows << std::endl;
    cv::Canny(img_gray,temp_grad, lower_thresh_val,high_thresh_val);
    temp_grad.copyTo( grad,temp_grad);
//    std::cout << img_gray.rows << " " << temp_grad.rows << " " << grad.rows << std::endl;
//    cv::Mat hola;
//    cv::bitwise_or(grad,temp_grad,grad);
//    grad = grad+temp_grad;
//    grad = hola;
            imshow("temp_grad",temp_grad);
                cv::waitKey();
    }
//    cv::convertScaleAbs( grad, grad );
//    cv::threshold(grad, grad, 128, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    return grad;
}

cv::Mat sobel(cv::Mat img)
{
    cv::Mat img_gray;
    if (img.channels()>1)
        cv::cvtColor(img,img_gray,CV_RGB2GRAY);
    else
        img.convertTo(img_gray,CV_8UC1);

    cv::Mat ch1, ch2, ch3;
    // "channels" is a vector of 3 Mat arrays:
    std::vector<cv::Mat> channels(3);

    // split img:
    cv::split(img, channels);
    // get the channels (dont forget they follow BGR order in OpenCV)
    ch1 = channels[0];
    ch2 = channels[1];
    ch3 = channels[2];

    cv::Mat grad(img.rows,img.cols,CV_16S,cv::Scalar(0));
    std::vector<cv::Mat> temp_grads(3);

    for (int i = 0; i < 3; i++)
    {
        img_gray = channels[i];
//        imshow("imggray", img_gray);

        int scale = 1;
        int delta = 0;
        /// Generate grad_x and grad_y
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;

        /// Gradient X
        //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
        cv::Sobel( img_gray, grad_x, CV_16S, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_x, abs_grad_x );

        /// Gradient Y
        //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
        cv::Sobel( img_gray, grad_y, CV_16S, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_y, abs_grad_y );

        /// Total Gradient (approximate)
        cv::Mat  temp_grad;
        cv::add(grad_x,grad_y,temp_grad);
        cv::add(temp_grad,grad,grad);
        //        grad = grad + grad_x + grad_y;
        //        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, temp_grad );
        //        cv::convertScaleAbs( temp_grad, temp_grad );
        //        cv::convertScaleAbs( grad, grad );
        //        imshow("temp_grad",temp_grad);
        //        imshow("grad",grad);
        //        cv::waitKey();
    }
    cv::convertScaleAbs( grad, grad );
//    cv::threshold(grad, grad, 128, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    return grad;
}

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinning(cv::Mat& im)
{
    im /= 255;

    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    }
    while (cv::countNonZero(diff) > 0);

    im *= 255;
}

Eigen::Vector4f getPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);//0.04

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);


    Eigen::Vector4f plane_vector(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);

    double angle = pcl::getAngle3D(plane_vector,Eigen::Vector4f(0,2,2,1));
    if (pcl::rad2deg(angle) < 90)
        plane_vector = -plane_vector;

    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud (cloud);
    ex.setIndices (inliers);
    ex.setNegative (false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ex.filter (*temp_cloud);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ> );
    *cloud = *temp_cloud;

    return plane_vector;
}

pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    if (cloud->width == 640)
    {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setNormalEstimationMethod (normal_estimator.AVERAGE_3D_GRADIENT);
        normal_estimator.setMaxDepthChangeFactor(0.02f);
        normal_estimator.setNormalSmoothingSize(10.f);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.compute(*normals);
    }
    else
    {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        if (radius < 1)
            normal_estimator.setRadiusSearch (radius);
        else
            normal_estimator.setKSearch( (int) radius);
        normal_estimator.compute (*normals);
    }

    return normals;
}

cv::Mat normalim(cv::Mat depth)
{
    cv::Mat depth_filtered;
    cv::bilateralFilter(depth,depth_filtered, 5, 10, 10, cv::BORDER_DEFAULT );
//    cv::GaussianBlur(depth, depth_filtered, cv::Size(3,3), 1, 1 );
//    cv::medianBlur(depth,depth_filtered,5);
//    depth_filtered = depth;
    cv::Mat normal(depth.rows,depth.cols,CV_8UC3);

    cv::Mat kernelH(3, 3, CV_32F, cv::Scalar(0));
    kernelH.at<float>(1,0) = 0.5f;
    kernelH.at<float>(1,2) = -0.5f;

    cv::Mat kernelV(3, 3, CV_32F, cv::Scalar(0));
    kernelV.at<float>(0,1) = -0.5f;
    kernelV.at<float>(2,1) = 0.5f;

    cv::Mat gradientx, gradienty;
    cv::filter2D(depth_filtered, gradientx,-1,kernelH,cv::Point(-1,-1),0,cv::BORDER_DEFAULT );
    cv::filter2D(depth_filtered, gradienty,-1,kernelV,cv::Point(-1,-1),0,cv::BORDER_DEFAULT );

    //compute the magnitude
    cv::Mat magnitude =  gradientx.mul(gradientx) +  gradienty.mul(gradienty);
    magnitude+=1.0;
    cv::sqrt(magnitude,magnitude);

    for (int i=0;i< magnitude.rows; i++)
        for(int j =0; j < magnitude.cols;j++){
            cv::Vec3b *cn = &normal.at<cv::Vec3b>(i,j );
            if (depth.at<float>(i,j) > 0)
            {
                float nx,ny,nz;

                nx = gradientx.at<float>(i,j)/magnitude.at<float>(i,j);
                ny  = -gradienty.at<float>(i,j)/magnitude.at<float>(i,j);
                nz = 1.0/magnitude.at<float>(i,j);
                cn->val[2] = (uchar) ((nx + 1.0) /2.0 * 255.0);
                cn->val[1] = (uchar) ((ny + 1.0) /2.0 * 255.0);
                cn->val[0] = (uchar) ((nz + 1.0) /2.0 * 255.0);
//                cn->val[2] = nx;
//                cn->val[1] = ny;
//                cn->val[0] = nz;
            }
            else
            {
                cn->val[2] = 0;
                cn->val[1] = 0;
                cn->val[0] = 0;
            }

        }

    return normal;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr compute3Dpoints(cv::Mat depth, cv::Mat K)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

    cv::Point3d point;

    cv::Mat res;
    cv::Mat Kinv = K.inv();

    pcl::PointXYZ p;

    for (std::size_t i=0; i < depth.rows; i++)
    {
        for (std::size_t j=0; j < depth.cols; j++)
        {
            point.x = j;
            point.y = i;
            point.z = 1;

            res = Kinv * cv::Mat( point, false);
            res.copyTo(cv::Mat ( point, false));

            if (depth.at<float>(i,j)>0.0)
            {
                point *= depth.at<float>(i,j)/1000.;

                p.x = point.x;
                p.y = point.y;
                p.z = point.z;

                pc->push_back(p);
            }
        }
    }
    return pc;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute3Dpoints(cv::Mat depth, cv::Mat K,cv::Mat color, cv::Mat K_color,cv::Mat R, cv::Mat T)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Point3d point;

    cv::Mat res;
    cv::Mat Kinv = K.inv();
    cv::Mat Rtrans;
    cv::transpose(R,Rtrans);

    pcl::PointXYZRGB pcolor;

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

                res = R *  cv::Mat( point, false) +  T ;
                res.copyTo(cv::Mat ( point, false));

                pcolor.x = point.x;
                pcolor.y = point.y;
                pcolor.z = point.z;

                res = K_color * cv::Mat( point,false);
                res.copyTo(cv::Mat ( point, false));

                v = (int) (point.x / point.z + .5);
                u = (int) (point.y / point.z + .5);

                if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
                {
                    uchar pr, pg, pb;

                    //Get RGB info
                    pb = color.at<cv::Vec3b>(u,v)[0];
                    pg = color.at<cv::Vec3b>(u,v)[1];
                    pr = color.at<cv::Vec3b>(u,v)[2];


                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);



                }
                else
                {
                    pcolor.r = 255;
                    pcolor.g = 255;
                    pcolor.b = 255;
                }
                pcColor->push_back(pcolor);
            }
        }
    }
    return pcColor;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr compute3DpointsOrganized(cv::Mat depth, cv::Mat K)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

    cv::Point3d point;

    cv::Mat res;
    cv::Mat Kinv = K.inv();

    pcl::PointXYZ p;

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

                p.x = point.x;
                p.y = point.y;
                p.z = point.z;

            }
            else
            {
                p.x = bad_point;
                p.y = bad_point;
                p.z = bad_point;
            }
            pc->push_back(p);
        }
    }
    pc->height = 480;
    pc->width = 640;
    pc->is_dense = false;
    return pc;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr compute3DpointsOrganized(cv::Mat depth, cv::Mat K,cv::Mat color, cv::Mat K_color,cv::Mat R, cv::Mat T)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Point3d point;

    cv::Mat res;
    cv::Mat Kinv = K.inv();

    pcl::PointXYZRGB pcolor;

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

                res = R *  cv::Mat( point, false) +  T ;
                res.copyTo(cv::Mat ( point, false));

                pcolor.x = point.x;
                pcolor.y = point.y;
                pcolor.z = point.z;

                res = K_color * cv::Mat( point,false);
                res.copyTo(cv::Mat ( point, false));

                v = (int) (point.x / point.z + .5);
                u = (int) (point.y / point.z + .5);

                if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
                {
                    uchar pr, pg, pb;

                    //Get RGB info
                    pb = color.at<cv::Vec3b>(u,v)[0];
                    pg = color.at<cv::Vec3b>(u,v)[1];
                    pr = color.at<cv::Vec3b>(u,v)[2];


                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);

                }
                else
                {
                    pcolor.r = 0;
                    pcolor.g = 255;
                    pcolor.b = 0;
                }
            }
            else
            {
                pcolor.x = bad_point;
                pcolor.y = bad_point;
                pcolor.z = bad_point;
                pcolor.r = 255;
                pcolor.g = 0;
                pcolor.b = 0;
            }

            pcColor->push_back(pcolor);
        }
    }
    pcColor->height = 480;
    pcColor->width = 640;
    pcColor->is_dense = false;
    return pcColor;
}


void projectionToOpenGLRBBottomLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
{
    float fu = K.at<double>(0,0);
    float fv = (float)K.at<double>(1, 1);
    float u0 = (float)K.at<double>(0, 2);
    float v0 = (float)K.at<double>(1, 2);

    float L = +(u0) * zNear / -fu;
    float T = +(v0) * zNear / fv;
    float R = -(w-u0) * zNear / -fu;
    float B = -(h-v0) * zNear / fv;

    std::fill_n(p,4*4,0);

    p[0*4+0] = 2 * zNear / (R-L);
    p[1*4+1] = 2 * zNear / (T-B);
    p[2*4+2] = -(zFar +zNear) / (zFar - zNear);
    p[2*4+0] = (R+L)/(R-L);
    p[2*4+1] = (T+B)/(T-B);
    p[2*4+3] = -1.0;
    p[3*4+2] =  -(2*zFar*zNear)/(zFar-zNear);

}

void projectionToOpenGLTopLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
{

    float fu = K.at<double>(0,0);
    float fv = (float)K.at<double>(1, 1);
    float u0 = (float)K.at<double>(0, 2);
    float v0 = (float)K.at<double>(1, 2);

    float L = -(u0) * zNear / fu;
    float R = +(w-u0) * zNear / fu;
    float T = -(v0) * zNear / fv;
    float B = +(h-v0) * zNear / fv;

    std::fill_n(p,4*4,0);

    p[0*4+0] = 2 * zNear / (R-L);
    p[1*4+1] = 2 * zNear / (T-B);
    p[2*4+0] = (R+L)/(L-R);
    p[2*4+1] = (T+B)/(B-T);
    p[2*4+2] = (zFar +zNear) / (zFar - zNear);
    p[2*4+3] = 1.0;
    p[3*4+2] =  (2*zFar*zNear)/(zNear - zFar);

}

void projectionToOpenGLBottomLeft( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
{

     float fu = K.at<double>(0,0);
    float fv = (float)K.at<double>(1, 1);
    float u0 = (float)K.at<double>(0, 2);
    float v0 = (float)K.at<double>(1, 2);

    float L = -(u0) * zNear / fu;
    float T = +(h-v0) * zNear / fv;
    float R = +(w-u0) * zNear / fu;
    float B = -(v0) * zNear / fv;

     std::fill_n(p,4*4,0);

    p[0*4+0] = 2 * zNear / (R-L);
    p[1*4+1] = 2 * zNear / (T-B);
    p[2*4+0] = (R+L)/(L-R);
    p[2*4+1] = (T+B)/(B-T);
    p[2*4+2] = (zFar +zNear) / (zFar - zNear);
    p[2*4+3] = 1.0;
    p[3*4+2] =  (2*zFar*zNear)/(zNear - zFar);


    Eigen::Matrix4d M(p);

    std::cout << "Matrix BL: \n" << M << std::endl;

}


void projectionToOpenGL( double p[16], const cv::Mat &K, int w, int h, float zNear, float zFar)
{

    float fu = K.at<double>(0,0);
    float fv = (float)K.at<double>(1, 1);
    float u0 = (float)K.at<double>(0, 2);
    float v0 = (float)K.at<double>(1, 2);

    float L = +(u0) * zNear / -fu;
    float T = +(v0) * zNear / fv;
    float R = -(w-u0) * zNear / -fu;
    float B = -(h-v0) * zNear / fv;


    p[0*4+0] = 2 * zNear / (R-L);
    p[1*4+1] = 2 * zNear / (T-B);
    p[2*4+2] = -(zFar +zNear) / (zFar - zNear);
    p[2*4+0] = (R+L)/(R-L);
    p[2*4+1] = (T+B)/(T-B);
    p[2*4+3] = -1.0;
    p[3*4+2] =  -(2*zFar*zNear)/(zFar-zNear);

}

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
