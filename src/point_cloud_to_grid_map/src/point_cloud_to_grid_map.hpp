/* soyeong made~ */

#ifndef POINTCLOUD2GRIDMAP_H_
#define POINTCLOUD2GRIDMAP_H_

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <velodyne_pointcloud/point_types.h>

#include "Eigen/Eigen"

#include <string>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ros/transport_hints.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/calib3d/calib3d.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <nav_msgs/OccupancyGrid.h>

class pointCloud2GridMap
{
    enum
    {
        GRID_UNKNOWN= -1,
        GRID_EMPTY= 0,
        GRID_OCCUPIED= 100
    };
    public:
        pointCloud2GridMap();
        ~pointCloud2GridMap();
    private:
        ros::NodeHandle m_nh;
        ros::Subscriber m_sub_filtered_point_cloud;

        void VelodyneCallback(const sensor_msgs::PointCloud2ConstPtr& input);
        
    private:
        ros::Publisher m_pub_occupancy_grid_map;
        std::string m_sub_point_cloud;

        double map_xMax;
        double map_xMin;
        double map_yMax;
        double map_yMin;

        double map_resolution;
        double safe_radius;

};

#endif