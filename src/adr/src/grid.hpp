#pragma once

#include "tf/transform_datatypes.h"
#include "nav_msgs/OccupancyGrid.h"

namespace Grid
{
    class OccupancyGridToPointCloud
    {
        nav_msgs::OccupancyGrid m_occupancy_grid;

    public:
        void setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid);

        void compute(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
    }

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
        std::tuple<uint32_t, uint32_t> m_cell;
        uint32_t m_radius;
        nav_msgs::OccupancyGrid m_occupancy_grid;

    public:
        void setCell(std::tuple<uint32_t, uint32_t> const &cell);

        void setRadius(uint32_t const &radius);

        void setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid);

        void compute(nav_msgs::OccupancyGrid &occupancy_grid);
    };
}