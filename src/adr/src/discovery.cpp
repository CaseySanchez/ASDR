#include "discovery.hpp"

void Discovery::Discovery::setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    m_occupancy_grid = occupancy_grid;
}

void Discovery::Discovery::setCell(std::tuple<uint32_t, uint32_t> const &cell)
{
    m_cell = cell;
}

void Discovery::Discovery::compute(std::optional<std::tuple<uint32_t, uint32_t>> &goal)
{
    // Search for the nearest `-1` in the occupancy grid

    auto const &[cell_x, cell_y] = m_cell;

    nav_msgs::OccupancyGrid visit_occupancy_grid;

    visit_occupancy_grid.info = m_occupancy_grid.info;

    visit_occupancy_grid.data.resize(visit_occupancy_grid.info.width * visit_occupancy_grid.info.height);

    std::fill(std::begin(visit_occupancy_grid.data), std::end(visit_occupancy_grid.data), -1);

    std::queue<std::tuple<uint32_t, uint32_t>> visit_queue;
    
    visit_queue.emplace(cell_x, cell_y);

    goal = std::nullopt;

    while (!visit_queue.empty()) {
        auto const &[x, y] = visit_queue.front();

        visit_queue.pop();

        {
            uint32_t const current_x = x + 1;
            uint32_t const current_y = y;

            uint32_t const index = current_x + current_y * m_occupancy_grid.info.width;

            if (index >= 0 && index < m_occupancy_grid.data.size()) {
                if (m_occupancy_grid.data[index] == -1 && current_x != cell_x && current_y != cell_y) {
                    goal = { current_x, current_y };
                    
                    break;
                }
                else if (m_occupancy_grid.data[index] == 0 && visit_occupancy_grid.data[index] == -1) {
                    visit_occupancy_grid.data[index] = 0;
                    
                    visit_queue.emplace(current_x, current_y);
                }
            }
        }
        
        {
            uint32_t const current_x = x;
            uint32_t const current_y = y + 1;

            uint32_t const index = current_x + current_y * m_occupancy_grid.info.width;

            if (index >= 0 && index < m_occupancy_grid.data.size()) {
                if (m_occupancy_grid.data[index] == -1 && current_x != cell_x && current_y != cell_y) {
                    goal = { current_x, current_y };
                    
                    break;
                }
                else if (m_occupancy_grid.data[index] == 0 && visit_occupancy_grid.data[index] == -1) {
                    visit_occupancy_grid.data[index] = 0;
                    
                    visit_queue.emplace(current_x, current_y);
                }
            }
        }

        {
            uint32_t const current_x = x - 1;
            uint32_t const current_y = y;

            uint32_t const index = current_x + current_y * m_occupancy_grid.info.width;

            if (index >= 0 && index < m_occupancy_grid.data.size()) {
                if (m_occupancy_grid.data[index] == -1 && current_x != cell_x && current_y != cell_y) {
                    goal = { current_x, current_y };
                    
                    break;
                }
                else if (m_occupancy_grid.data[index] == 0 && visit_occupancy_grid.data[index] == -1) {
                    visit_occupancy_grid.data[index] = 0;
                    
                    visit_queue.emplace(current_x, current_y);
                }
            }
        }

        {
            uint32_t const current_x = x;
            uint32_t const current_y = y - 1;

            uint32_t const index = current_x + current_y * m_occupancy_grid.info.width;

            if (index >= 0 && index < m_occupancy_grid.data.size()) {
                if (m_occupancy_grid.data[index] == -1 && current_x != cell_x && current_y != cell_y) {
                    goal = { current_x, current_y };
                    
                    break;
                }
                else if (m_occupancy_grid.data[index] == 0 && visit_occupancy_grid.data[index] == -1) {
                    visit_occupancy_grid.data[index] = 0;
                    
                    visit_queue.emplace(current_x, current_y);
                }
            }
        }
    }
}

void Discovery::TransformToCell::setTransform(tf::StampedTransform const &transform)
{
    m_transform = transform;
}

void Discovery::TransformToCell::setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    m_occupancy_grid = occupancy_grid;
}

void Discovery::TransformToCell::compute(std::tuple<uint32_t, uint32_t> &cell)
{
    Eigen::Vector3d const transform_position(m_transform.getOrigin().x(), m_transform.getOrigin().y(), m_transform.getOrigin().z());

    Eigen::Vector3d const origin_position(m_occupancy_grid.info.origin.position.x, m_occupancy_grid.info.origin.position.y, m_occupancy_grid.info.origin.position.z);
    Eigen::Quaterniond const origin_orientation(m_occupancy_grid.info.origin.orientation.w, m_occupancy_grid.info.origin.orientation.x, m_occupancy_grid.info.origin.orientation.y, m_occupancy_grid.info.origin.orientation.z);

    Eigen::Vector3d const position = origin_orientation.inverse() * (transform_position - origin_position);

    uint32_t const cell_x = static_cast<uint32_t>(position.x() / m_occupancy_grid.info.resolution);
    uint32_t const cell_y = static_cast<uint32_t>(position.y() / m_occupancy_grid.info.resolution);

    cell = { cell_x, cell_y };
}

void Discovery::Circle::setCell(std::tuple<uint32_t, uint32_t> const &cell)
{
    m_cell = cell;
}

void Discovery::Circle::setRadius(uint32_t const &radius)
{
    m_radius = radius;
}

void Discovery::Circle::setOccupancyGrid(nav_msgs::OccupancyGrid const &occupancy_grid)
{
    m_occupancy_grid = occupancy_grid;
}

void Discovery::Circle::compute(nav_msgs::OccupancyGrid &occupancy_grid)
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