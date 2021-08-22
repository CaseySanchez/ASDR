#pragma once

#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.10/pcl/filters/radius_outlier_removal.h>
#include <pcl-1.10/pcl/surface/concave_hull.h>

#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "mandoline.hpp"

class PathPlannerNode : public ros::NodeHandle
{
    ros::Subscriber m_occupancy_grid_subscriber;
    ros::Publisher m_path_publisher;

public:
    PathPlannerNode();

    void onOccupancyGrid(nav_msgs::OccupancyGrid::ConstPtr const &occupancy_grid);
};