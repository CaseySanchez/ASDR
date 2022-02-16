#include "discovery_node.hpp"

DiscoveryNode::DiscoveryNode(ros::NodeHandle const &node_handle) : 
    m_node_handle { node_handle }
{
    m_occupancy_grid_subscriber = m_node_handle.subscribe(ros::names::resolve("/rtabmap/grid_map"), 1, &DiscoveryNode::onOccupancyGrid, this);

    m_discover_server = m_node_handle.advertiseService(ros::names::resolve("discover"), &DiscoveryNode::onDiscover, this);
}

void DiscoveryNode::onOccupancyGrid(nav_msgs::OccupancyGrid::ConstPtr const &occupancy_grid)
{
    m_occupancy_grid = *occupancy_grid;
}

bool DiscoveryNode::onDiscover(discovery::discover::Request &request, discovery::discover::Response &response)
{
    if (m_occupancy_grid.has_value()) {
        nav_msgs::OccupancyGrid const occupancy_grid = m_occupancy_grid.value();

        tf::StampedTransform transform;

        m_transform_listener.waitForTransform("/map", "/camera_link", ros::Time(0), ros::Duration(1.0));
        m_transform_listener.lookupTransform("/map", "/camera_link", ros::Time(0), transform);

        Eigen::Vector3d const transform_position(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

        Eigen::Vector3d const origin_position(occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y, occupancy_grid.info.origin.position.z);
        Eigen::Quaterniond const origin_orientation(occupancy_grid.info.origin.orientation.w, occupancy_grid.info.origin.orientation.x, occupancy_grid.info.origin.orientation.y, occupancy_grid.info.origin.orientation.z);

        Eigen::Vector3d const position = origin_orientation.inverse() * (transform_position - origin_position);

        uint32_t const cell_x = static_cast<uint32_t>(position.x() / occupancy_grid.info.resolution);
        uint32_t const cell_y = static_cast<uint32_t>(position.y() / occupancy_grid.info.resolution);

        std::tuple<uint32_t, uint32_t> const cell = { cell_x, cell_y };

        std::optional<std::tuple<uint32_t, uint32_t>> goal;

        Discovery discovery;

        discovery.setOccupancyGrid(occupancy_grid);
        discovery.setCell(cell);
        discovery.compute(goal);

        // No value means no goal was found

        if (goal.has_value()) {
            auto const &[goal_x, goal_y] = goal.value();

            Eigen::Vector3d const position(static_cast<float>(goal_x) * occupancy_grid.info.resolution, static_cast<float>(goal_y) * occupancy_grid.info.resolution, 0.0);

            Eigen::Vector3d const origin_position(occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y, occupancy_grid.info.origin.position.z);
            Eigen::Quaterniond const origin_orientation(occupancy_grid.info.origin.orientation.w, occupancy_grid.info.origin.orientation.x, occupancy_grid.info.origin.orientation.y, occupancy_grid.info.origin.orientation.z);

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

            response.pose_stamped = pose_stamped;

            response.status = discovery::discover::Response::SUCCESS;
        }
        else {
            response.status = discovery::discover::Response::FAILURE;
        }

        return true;
    }

    return false;
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "discovery_node");

        ros::NodeHandle node_handle("~");

        DiscoveryNode discovery_node(node_handle);
        
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