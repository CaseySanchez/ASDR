#include "path_planner_node.hpp"

PathPlannerNode::PathPlannerNode() : ros::NodeHandle("")
{
    m_occupancy_grid_subscriber = subscribe("/rtabmap/grid_map", 1, &PathPlannerNode::onOccupancyGrid, this);

    m_path_publisher = advertise<nav_msgs::Path>("/path", 1);
}

void PathPlannerNode::onOccupancyGrid(nav_msgs::OccupancyGrid::ConstPtr const &occupancy_grid)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_extruded(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_sliced(new pcl::PointCloud<pcl::PointXYZ>);

    for (uint32_t index = 0; index < occupancy_grid->info.height * occupancy_grid->info.width; ++index) {
        if (occupancy_grid->data[index] == 0) {
            uint32_t x = index % occupancy_grid->info.width;
            uint32_t y = index / occupancy_grid->info.width;

            Eigen::Vector3d position(static_cast<float>(x) * occupancy_grid->info.resolution, static_cast<float>(y) * occupancy_grid->info.resolution, 0.0);

            Eigen::Vector3d origin_position(occupancy_grid->info.origin.position.x, occupancy_grid->info.origin.position.y, occupancy_grid->info.origin.position.z);
            Eigen::Quaterniond origin_orientation(occupancy_grid->info.origin.orientation.w, occupancy_grid->info.origin.orientation.x, occupancy_grid->info.origin.orientation.y, occupancy_grid->info.origin.orientation.z);

            Eigen::Vector3d point = origin_orientation * position + origin_position;
            
            point_cloud->points.emplace_back(point.x(), point.y(), point.z());
        }
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    Mandoline::Extrude extrude;
    Mandoline::Slice slice;

    statistical_outlier_removal.setInputCloud(point_cloud);
    statistical_outlier_removal.setMeanK(1000);
    statistical_outlier_removal.setStddevMulThresh(0.01);
    statistical_outlier_removal.filter(*point_cloud_filtered);
    
    concave_hull.setAlpha(0.1);
    concave_hull.setInputCloud(point_cloud_filtered);
    concave_hull.reconstruct(*point_cloud_hull);

    /*extrude.setInputCloud(point_cloud_hull);
    extrude.setDistance(-0.25f);
    extrude.compute(*point_cloud_extruded);*/

    slice.setInputCloud(point_cloud_hull);
    slice.setSpacing(0.25f);
    slice.compute(*point_cloud_sliced);

    ROS_INFO("size = %ld\n", point_cloud_sliced->points.size());

    /*
    sensor_msgs::PointCloud2 point_cloud_msg;

    pcl::toROSMsg(*point_cloud_extruded.get(), point_cloud_msg);

    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_msg.header.frame_id = "map";

    m_path_publisher.publish(point_cloud_msg);
    */

    nav_msgs::Path path;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    for (size_t index = 0; index < point_cloud_sliced->points.size(); ++index) {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";

        pose_stamped.pose.position.x = point_cloud_sliced->points[index].x;
        pose_stamped.pose.position.y = point_cloud_sliced->points[index].y;
        pose_stamped.pose.position.z = 0.0f;

        pose_stamped.pose.orientation.x = 0.0f;
        pose_stamped.pose.orientation.y = 0.0f;
        pose_stamped.pose.orientation.z = 0.0f;
        pose_stamped.pose.orientation.w = 1.0f;

        path.poses.push_back(pose_stamped);
    }

    m_path_publisher.publish(path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");

    PathPlannerNode path_planner_node;
    
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}