#include "coverage_path_planner_node.hpp"

CoveragePathPlannerNode::CoveragePathPlannerNode(ros::NodeHandle &node_handle) : m_node_handle(node_handle)
{
    std::string make_plan_service;

    if (!m_node_handle.getParam("make_plan_service", make_plan_service)) {
        throw std::runtime_error("make_plan_service not provided");
    }

    m_point_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    m_occupancy_grid_subscriber = m_node_handle.subscribe("/rtabmap/cloud_ground", 1, &CoveragePathPlannerNode::onCloudGround, this);

    m_make_plan_server = m_node_handle.advertiseService(make_plan_service, &CoveragePathPlannerNode::onMakePlan, this);
}

void CoveragePathPlannerNode::onCloudGround(sensor_msgs::PointCloud2::ConstPtr const &point_cloud)
{
    pcl::fromROSMsg(*point_cloud, *m_point_cloud);
}

bool CoveragePathPlannerNode::onMakePlan(coverage_path_planner::make_plan::Request &request, coverage_path_planner::make_plan::Response &response)
{
    if (!m_point_cloud->empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_extruded(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_sliced(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;

        statistical_outlier_removal.setInputCloud(m_point_cloud);
        statistical_outlier_removal.setMeanK(1000);
        statistical_outlier_removal.setStddevMulThresh(0.01);
        statistical_outlier_removal.filter(*point_cloud_filtered);
        
        pcl::ConcaveHull<pcl::PointXYZ> concave_hull;

        concave_hull.setAlpha(0.1);
        concave_hull.setInputCloud(point_cloud_filtered);
        concave_hull.reconstruct(*point_cloud_hull);

        Mandoline::Extrude extrude;

        extrude.setInputCloud(point_cloud_hull);
        extrude.setDistance(-0.5f);
        extrude.compute(*point_cloud_extruded);

        Mandoline::Slice slice;

        slice.setInputCloud(point_cloud_extruded);
        slice.setSpacing(0.25f);
        slice.compute(*point_cloud_sliced);

        nav_msgs::Path plan;

        plan.header.stamp = ros::Time::now();
        plan.header.frame_id = "map";

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

            plan.poses.push_back(pose_stamped);
        }

        response.plan = plan;

        return true;
    }

    return false;
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "coverage_path_planner_node");

        ros::NodeHandle node_handle("~");

        CoveragePathPlannerNode coverage_path_planner_node(node_handle);
        
        while (ros::ok()) {
            ros::spinOnce();
        }

        return 0;
    }
    catch (std::exception const &exception) {
        ROS_ERROR("%s", exception.what());

        return 1;
    }
}