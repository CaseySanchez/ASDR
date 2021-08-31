#include "path_planner_node.hpp"

PathPlannerNode::PathPlannerNode() : ros::NodeHandle("")
{
    m_occupancy_grid_subscriber = subscribe("/rtabmap/grid_map", 1, &PathPlannerNode::onOccupancyGrid, this);

    m_path_publisher = advertise<nav_msgs::Path>("/path", 1);
}

void PathPlannerNode::onOccupancyGrid(nav_msgs::OccupancyGrid::ConstPtr const &occupancy_grid)
{
    std::optional<geometry_msgs::PoseStamped> goal = discover(*occupancy_grid);

    nav_msgs::Path path = plan(*occupancy_grid);

    m_path_publisher.publish(path);
}

nav_msgs::Path PathPlannerNode::plan(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_sliced(new pcl::PointCloud<pcl::PointXYZ>);

    for (uint32_t index = 0; index < occupancy_grid.info.height * occupancy_grid.info.width; ++index) {
        if (occupancy_grid.data[index] == 0) {
            uint32_t const x = index % occupancy_grid.info.width;
            uint32_t const y = index / occupancy_grid.info.width;

            Eigen::Vector3d const position(static_cast<float>(x) * occupancy_grid.info.resolution, static_cast<float>(y) * occupancy_grid.info.resolution, 0.0);

            Eigen::Vector3d const origin_position(occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y, occupancy_grid.info.origin.position.z);
            Eigen::Quaterniond const origin_orientation(occupancy_grid.info.origin.orientation.w, occupancy_grid.info.origin.orientation.x, occupancy_grid.info.origin.orientation.y, occupancy_grid.info.origin.orientation.z);

            Eigen::Vector3d const point = origin_orientation * position + origin_position;
            
            point_cloud->points.emplace_back(point.x(), point.y(), point.z());
        }
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;

    statistical_outlier_removal.setInputCloud(point_cloud);
    statistical_outlier_removal.setMeanK(1000);
    statistical_outlier_removal.setStddevMulThresh(0.01);
    statistical_outlier_removal.filter(*point_cloud_filtered);
    
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;

    concave_hull.setAlpha(0.1);
    concave_hull.setInputCloud(point_cloud_filtered);
    concave_hull.reconstruct(*point_cloud_hull);

    Mandoline::Slice slice;

    slice.setInputCloud(point_cloud_hull);
    slice.setSpacing(0.25f);
    slice.compute(*point_cloud_sliced);

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

    return path;
}

std::optional<geometry_msgs::PoseStamped> PathPlannerNode::discover(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    tf::StampedTransform transform;

    m_transform_listener.waitForTransform("/map", "/camera_link", ros::Time(0), ros::Duration(1.0));
    m_transform_listener.lookupTransform("/map", "/camera_link", ros::Time(0), transform);

    std::tuple<uint32_t, uint32_t> cell;

    Discovery::TransformToCell transform_to_cell;

    transform_to_cell.setTransform(transform);
    transform_to_cell.setOccupancyGrid(occupancy_grid);
    transform_to_cell.compute(cell);

    nav_msgs::OccupancyGrid circle_occupancy_grid;

    Discovery::Circle circle;

    circle.setRadius(20);
    circle.setCell(cell);
    circle.setOccupancyGrid(occupancy_grid);
    circle.compute(circle_occupancy_grid);

    std::optional<std::tuple<uint32_t, uint32_t>> goal;

    Discovery::Discovery discovery;

    discovery.setOccupancyGrid(circle_occupancy_grid);
    discovery.setCell(cell);
    discovery.compute(goal);

    // Convert from indices to pose

    std::optional<geometry_msgs::PoseStamped> pose = std::nullopt;

    // No value means no goal was found

    if (goal.has_value()) {
        auto const &[goal_x, goal_y] = goal.value();

        Eigen::Vector3d const position(static_cast<float>(goal_x) * circle_occupancy_grid.info.resolution, static_cast<float>(goal_y) * circle_occupancy_grid.info.resolution, 0.0);

        Eigen::Vector3d const origin_position(circle_occupancy_grid.info.origin.position.x, circle_occupancy_grid.info.origin.position.y, circle_occupancy_grid.info.origin.position.z);
        Eigen::Quaterniond const origin_orientation(circle_occupancy_grid.info.origin.orientation.w, circle_occupancy_grid.info.origin.orientation.x, circle_occupancy_grid.info.origin.orientation.y, circle_occupancy_grid.info.origin.orientation.z);

        Eigen::Vector3d const point = origin_orientation * position + origin_position;

        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";

        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();

        pose_stamped.pose.orientation.x = 0.0f;
        pose_stamped.pose.orientation.y = 0.0f;
        pose_stamped.pose.orientation.z = 0.0f;
        pose_stamped.pose.orientation.w = 1.0f;

        pose = pose_stamped;
    }

    return pose;
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