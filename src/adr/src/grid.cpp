#include "grid.hpp"

void Grid::OccupancyGridToPointCloud::setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    m_occupancy_grid = occupancy_grid;
}

void Grid::OccupancyGridToPointCloud::compute(pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{
    for (uint32_t index = 0; index < m_occupancy_grid.info.height * m_occupancy_grid.info.width; ++index) {
        if (m_occupancy_grid.data[index] == 0) {
            uint32_t const x = index % m_occupancy_grid.info.width;
            uint32_t const y = index / m_occupancy_grid.info.width;

            Eigen::Vector3d const position(static_cast<float>(x) * m_occupancy_grid.info.resolution, static_cast<float>(y) * m_occupancy_grid.info.resolution, 0.0);

            Eigen::Vector3d const origin_position(m_occupancy_grid.info.origin.position.x, m_occupancy_grid.info.origin.position.y, m_occupancy_grid.info.origin.position.z);
            Eigen::Quaterniond const origin_orientation(m_occupancy_grid.info.origin.orientation.w, m_occupancy_grid.info.origin.orientation.x, m_occupancy_grid.info.origin.orientation.y, m_occupancy_grid.info.origin.orientation.z);

            Eigen::Vector3d const point = origin_orientation * position + origin_position;
            
            point_cloud.points.emplace_back(point.x(), point.y(), point.z());
        }
    }
}

void Grid::TransformToCell::setTransform(tf::StampedTransform const &transform)
{
    m_transform = transform;
}

void Grid::TransformToCell::setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    m_occupancy_grid = occupancy_grid;
}

void Grid::TransformToCell::compute(std::tuple<uint32_t, uint32_t> &cell)
{
    Eigen::Vector3d const transform_position(m_transform.getOrigin().x(), m_transform.getOrigin().y(), m_transform.getOrigin().z());

    Eigen::Vector3d const origin_position(m_occupancy_grid.info.origin.position.x, m_occupancy_grid.info.origin.position.y, m_occupancy_grid.info.origin.position.z);
    Eigen::Quaterniond const origin_orientation(m_occupancy_grid.info.origin.orientation.w, m_occupancy_grid.info.origin.orientation.x, m_occupancy_grid.info.origin.orientation.y, m_occupancy_grid.info.origin.orientation.z);

    Eigen::Vector3d const position = origin_orientation.inverse() * (transform_position - origin_position);

    uint32_t const cell_x = static_cast<uint32_t>(position.x() / m_occupancy_grid.info.resolution);
    uint32_t const cell_y = static_cast<uint32_t>(position.y() / m_occupancy_grid.info.resolution);

    cell = { cell_x, cell_y };
}

void Grid::Circle::setCell(std::tuple<uint32_t, uint32_t> const &cell)
{
    m_cell = cell;
}

void Grid::Circle::setRadius(uint32_t const &radius)
{
    m_radius = radius;
}

void Grid::Circle::setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    m_occupancy_grid = occupancy_grid;
}

void Grid::Circle::compute(nav_msgs::OccupancyGrid &occupancy_grid)
{
    // Populate occupancy grid with `0`s where the robot is in a circle

    occupancy_grid = m_occupancy_grid;

    auto const &[cell_x, cell_y] = m_cell;

    for (int32_t x = -m_radius; x <= m_radius; ++x) {
        for (int32_t y = -m_radius; y <= m_radius; ++y) {
            uint32_t const index = (cell_x + x) + (cell_y + y) * m_occupancy_grid.info.width;

            if (index >= 0 && index < m_occupancy_grid.data.size()) {
                float const magnitude_squared = static_cast<float>(x * x) + static_cast<float>(y * y);

                if (m_occupancy_grid.data[index] == -1 && magnitude_squared <= static_cast<float>(m_radius * m_radius)) {
                    occupancy_grid.data[index] = 0;
                }
            }
        }
    }        
}