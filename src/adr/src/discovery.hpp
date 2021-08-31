#pragma once

#include <tuple>
#include <optional>
#include <queue>

#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "ros/console.h"

#include "tf/transform_listener.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

namespace Discovery
{
    class Discovery
    {
        nav_msgs::OccupancyGrid m_occupancy_grid;
        std::tuple<uint32_t, uint32_t> m_cell;

    public:
        void setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid);

        void setCell(std::tuple<uint32_t, uint32_t> const &cell);

        void compute(std::optional<std::tuple<uint32_t, uint32_t>> &goal);
    };

    class TransformToCell
    {
        tf::StampedTransform m_transform;
        nav_msgs::OccupancyGrid m_occupancy_grid;

    public:
        void setTransform(tf::StampedTransform const &transform);

        void setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid);

        void compute(std::tuple<uint32_t, uint32_t> &cell);
    };

    class Circle
    {
        nav_msgs::OccupancyGrid m_occupancy_grid;
        std::tuple<uint32_t, uint32_t> m_cell;
        uint32_t m_radius;

    public:
        void setCell(std::tuple<uint32_t, uint32_t> const &cell);

        void setRadius(uint32_t const &radius);

        void setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid);

        void compute(nav_msgs::OccupancyGrid &occupancy_grid);
    };
}