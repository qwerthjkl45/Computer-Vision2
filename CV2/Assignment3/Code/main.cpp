/*
 * main.cpp
 *
 *  Created on: 28 May 2016
 *      Author: Minh Ngo @ 3DUniversum
 */
 
#include <cmath>
#include <iostream>
#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/texture_mapping.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/common/common.h>
#include <pcl/surface/texture_mapping.h>

#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>

#include "Frame3D/Frame3D.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr mat2IntegralPointCloud(const cv::Mat& depth_mat, const float focal_length, const float max_depth) {
    // This function converts a depth image to a point cloud
    assert(depth_mat.type() == CV_16U);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    const int half_width = depth_mat.cols / 2;
    const int half_height = depth_mat.rows / 2;
    const float inv_focal_length = 1.0 / focal_length;
    point_cloud->points.reserve(depth_mat.rows * depth_mat.cols);
    for (int y = 0; y < depth_mat.rows; y++) {
        for (int x = 0; x < depth_mat.cols; x++) {
            float z = depth_mat.at<ushort>(cv:: Point(x, y)) * 0.001;
            if (z < max_depth && z > 0) {
                point_cloud->points.emplace_back(static_cast<float>(x - half_width)  * z * inv_focal_length,
                                                 static_cast<float>(y - half_height) * z * inv_focal_length,
                                                 z);
            } else {
                point_cloud->points.emplace_back(x, y, NAN);
            }
        }
    }
    point_cloud->width = depth_mat.cols;
    point_cloud->height = depth_mat.rows;
    return point_cloud;
}


pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // This function computes normals given a point cloud
    // !! Please note that you should remove NaN values from the pointcloud after computing the surface normals.
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>); // Output datasets
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);
    pcl::copyPointCloud(*cloud, *cloud_normals);
    return cloud_normals;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const Eigen::Matrix4f& transform) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}


template<class T>
typename pcl::PointCloud<T>::Ptr transformPointCloudNormals(typename pcl::PointCloud<T>::Ptr cloud, const Eigen::Matrix4f& transform) {
    typename pcl::PointCloud<T>::Ptr transformed_cloud(new typename pcl::PointCloud<T>());
    pcl::transformPointCloudWithNormals(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergingPointClouds(Frame3D frames[]) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    for (int i = 0; i < 8; i++) {
        std::cout << boost::format("Merging frame %d") % i << std::endl;

        Frame3D frame = frames[i];
        cv::Mat depthImage = frame.depth_image_;
        double focalLength = frame.focal_length_;
        const Eigen::Matrix4f cameraPose = frame.getEigenTransform();

        // TODO(Student): Merge the i-th frame using predicted camera pose
        // to the global point cloud. ~ 20 lines.
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = mat2IntegralPointCloud(depthImage, focalLength, 1.5);
        pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normals = computeNormals(point_cloud);
        
        //remove NaN values from the pointcloud after computing the surface normals
        pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normals_after_removal(new pcl::PointCloud<pcl::PointNormal>());
        point_cloud_with_normals_after_removal->points.reserve(point_cloud_with_normals->size());
        
        for (int idx = 0; idx < point_cloud_with_normals->size(); idx++) {
        
            if (std::isnan(point_cloud_with_normals->points[idx].normal_z) ||\
             std::isnan(point_cloud_with_normals->points[idx].normal_z) || \
             std::isnan(point_cloud_with_normals->points[idx].normal_z)){
                continue;
            }
            
            point_cloud_with_normals_after_removal->push_back(point_cloud_with_normals->points[idx]);
        }
        
        //transfrom point clouds
        point_cloud_with_normals = transformPointCloudNormals<pcl::PointNormal>(point_cloud_with_normals_after_removal, cameraPose);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelCloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::copyPointCloud(*point_cloud_with_normals, *modelCloud_tmp);
        *modelCloud += *modelCloud_tmp;
    }  
    

    return modelCloud;
}


cv::Mat recordCoordinate(const pcl::PointCloud<pcl::PointXYZRGB>& pts, Frame3D& frame) {

    double focal_length = frame.focal_length_;
    double sizeX = frame.depth_image_.cols;
    double sizeY = frame.depth_image_.rows;
    double cx = sizeX / 2.0;
    double cy = sizeY / 2.0;
    cv::Mat pointCoordinate(sizeY, sizeX, CV_32FC3, cv::Vec3f(1000, 1000, 1000));
    
    //calcualte point coordinate corresponds to the pixel in the frame    
    for ( const pcl::PointXYZRGB& pt : pts) {
        
         
        int col =  std::round(focal_length * (pt.x / pt.z) + cx);
        int row =  std::round(focal_length * (pt.y / pt.z) + cy);
        
        if (row < 0 || row >= sizeY || col < 0 || col >= sizeX)
            continue;
        
        //becuase use round, some points will assign two times
        cv::Vec3f& point = pointCoordinate.at<cv::Vec3f>(row, col);
        if (point[2] > pt.z) {
            point[0] = pt.x;
            point[1] = pt.y;
            point[2] = pt.z;
        }
                 
    }
    
    return pointCoordinate;
    
}



pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergingPointCloudsWithTexture(Frame3D frames[], pcl::PolygonMesh mesh) {

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(mesh.cloud, *point_cloud);
    std::vector<pcl::Vertices> polygons = mesh.polygons;
   
    for (int i = 0; i < 8; i++) {
    
        Frame3D frame = frames[i];
        cv::Mat depthImage = frame.depth_image_;
        double focalLength = frame.focal_length_;
        // compute image center and dimension
        double sizeX = frame.depth_image_.cols;
        double sizeY = frame.depth_image_.rows;
        double cx = sizeX / 2.0;
        double cy = sizeY / 2.0;
        
        const Eigen::Matrix4f cameraPose = frame.getEigenTransform();

        // TODO(Student): The same as mergingPointClouds but now with texturing. ~ 50 lines.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        transformed_point_cloud = transformPointCloud(point_cloud, cameraPose.inverse());
        cv::Mat visiblePoints = recordCoordinate(*transformed_point_cloud, frame);
        
        //set camera parameter
        pcl::texture_mapping::Camera cam;
        cam.focal_length = focalLength;
        cam.center_w = cx;
        cam.center_h = cy;
        cam.width = sizeX;
        cam.height = sizeY;
        
        for (auto polygon = polygons.begin(); polygon != polygons.end(); ++polygon){
        
            const pcl::PointXYZRGB& point = transformed_point_cloud->at(polygon->vertices[0]);
            int u = std::round(focalLength * (point.x / point.z) + cx);
            int v = std::round(focalLength * (point.y / point.z) + cy);
            
            //check if the point is visible to the camera now            
            float u_normalize = static_cast<float> ((focalLength * (point.x / point.z) + cx) / sizeX); //horizontal
            float v_normalize = static_cast<float> ((focalLength * (point.y / point.z) + cy) / sizeY); //horizontal
            
            if (u_normalize < 0 || u_normalize >= 1 || v_normalize < 0 || v_normalize >= 1)
                continue;
            
            
            float thread = 0.0000001;
            cv::Vec3f visiblePoint = visiblePoints.at<cv::Vec3f>(v, u);
            if (std::abs(visiblePoint[0] - point.x) > thread\
                || std::abs(visiblePoint[1] - point.y) > thread\
                || std::abs(visiblePoint[2] - point.z) > thread)
                continue;
                       
            
            int x = std::floor(frame.rgb_image_.cols * u_normalize);
            int y = std::floor(frame.rgb_image_.rows * v_normalize);
            
            //each polygon has three vertices, filling the color in the traingle
            for (int v_idx = 0; v_idx < 3; ++v_idx) {
                pcl::PointXYZRGB& p = point_cloud->at(polygon->vertices[v_idx]);
                cv::Vec3b frame_color = frame.rgb_image_.at<cv::Vec3b>(y, x);
                if (p.r != 0 && p.g != 0 && p.b != 0)
                    continue;
                p.b = frame_color[0];
                p.g = frame_color[1];
                p.r = frame_color[2];
            }
        }
        
    }
    pcl::copyPointCloud(*point_cloud, *modelCloud);
    return modelCloud;
}

// Different methods of constructing mesh
enum CreateMeshMethod { PoissonSurfaceReconstruction = 0, MarchingCubes = 1};

// Create mesh from point cloud using one of above methods
pcl::PolygonMesh createMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud, CreateMeshMethod method) {
    std::cout << "Creating meshes" << std::endl;
    int test = static_cast<int>(method);
    std::cout<<method<<std::endl;
    
    // The variable for the constructed mesh
    pcl::PolygonMesh triangles;    
    switch (method) {
        case MarchingCubes:
            std::cout<<"MarchingCubes"<<std::endl;
            
			pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
            tree->setInputCloud(pointCloud);

            // create the polygonmesh variable
            pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> MarchCubes;

            // get result
            MarchCubes.setInputCloud(pointCloud);
            MarchCubes.setSearchMethod(tree);
            MarchCubes.reconstruct(triangles);
            
            break;
        case PoissonSurfaceReconstruction:
            // TODO(Student): Call Poisson Surface Reconstruction. ~ 5 lines.
            std::cout<<"poisson"<<std::endl;
            pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
            poisson.setDepth(10);
            poisson.setInputCloud(pointCloud);
            poisson.reconstruct(triangles);   
            break;
        
    }
    return triangles;
}


int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cout << "./final [3DFRAMES PATH] [RECONSTRUCTION MODE] [TEXTURE_MODE]" << std::endl;

        return 0;
    }

    const CreateMeshMethod reconMode = static_cast<CreateMeshMethod>(std::stoi(argv[2]));

    // Loading 3D frames
    Frame3D frames[8];
    for (int i = 0; i < 8; ++i) {
        frames[i].load(boost::str(boost::format("%s/%05d.3df") % argv[1] % i));
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr texturedCloud;
    pcl::PolygonMesh triangles;

    if (argv[3][0] == 't') {
        // SECTION 4: Coloring 3D Model
        // Create one point cloud by merging all frames with texture using
        // the rgb images from the frames
        texturedCloud = mergingPointClouds(frames);
        //texturedCloud = mergingPointCloudsWithTexture(frames);

        // Create a mesh from the textured cloud using a reconstruction method,
        // Poisson Surface or Marching Cubes
        triangles = createMesh(texturedCloud, reconMode);
        texturedCloud = mergingPointCloudsWithTexture(frames, triangles);
        triangles = createMesh(texturedCloud, reconMode);
    } else {
        // SECTION 3: 3D Meshing & Watertighting

        // Create one point cloud by merging all frames with texture using
        // the rgb images from the frames
        texturedCloud = mergingPointClouds(frames);

        
        // Create a mesh from the textured cloud using a reconstruction method,
        // Poisson Surface or Marching Cubes
        triangles = createMesh(texturedCloud, reconMode);
        std::cout<<"create triangles"<<std::endl;
    }

    // Sample code for visualization.

    // Show viewer
    std::cout << "Finished texturing" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Add colored point cloud to viewer, because it does not support colored meshes
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(texturedCloud);
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(texturedCloud, rgb, "cloud");

    // Add mesh
    
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addPolygonMesh(triangles, "meshes", 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(1,1,-2,0,0,0,0); 
    
    std::vector<pcl::visualization::Camera> cam; 

    //Save the position of the camera           
    
    

    // Keep viewer open
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        viewer->getCameras(cam); 

    //Print recorded points on the screen: 
    /*cout << "Cam: " << endl 
                 << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
                 << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
                 << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;*/
    
    }


    return 0;
}
