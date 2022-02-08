#pragma once

#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.10/pcl/filters/radius_outlier_removal.h>
#include <pcl-1.10/pcl/surface/concave_hull.h>

#include "pcl_conversions/pcl_conversions.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"

#include "coverage_path_planner/make_plan.h"

#include "mandoline.hpp"

class CoveragePathPlannerNode
{
    ros::NodeHandle m_node_handle;
    
    ros::Subscriber m_point_cloud_subscriber;

    ros::ServiceServer m_make_plan_server;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_point_cloud;

    float m_extrude_distance;
    float m_slice_spacing;

public:
    CoveragePathPlannerNode(ros::NodeHandle &node_handle);

private:
    void onPointCloud(sensor_msgs::PointCloud2::ConstPtr const &point_cloud);

    bool onMakePlan(coverage_path_planner::make_plan::Request &request, coverage_path_planner::make_plan::Response &response);
};