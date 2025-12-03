#ifndef GRID_FAST_SLAM_HPP_
#define GRID_FAST_SLAM_HPP_

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msgs/msg/delta_odom.hpp"

namespace grid_fastslam
{

    // ==========================================
    // 1. CONSTANTS (From robot_functions.py)
    // ==========================================
    constexpr int MAP_HEIGHT = 100;
    constexpr int MAP_WIDTH = 100;
    constexpr double RESOLUTION = 0.075;
    constexpr double OX = -(MAP_WIDTH * RESOLUTION) / 2.0;
    constexpr double OY = -(MAP_HEIGHT * RESOLUTION) / 2.0;

    // Probability parameters
    constexpr double P_OCC = 0.7;
    constexpr double P_FREE = 0.3;

    // Log-odds increments (computed at compile time or initialization)
    const double L_OCC = std::log(P_OCC / (1.0 - P_OCC));
    const double L_FREE = std::log(P_FREE / (1.0 - P_FREE));
    constexpr double L_MIN = -5.0;
    constexpr double L_MAX = 5.0;

    // ==========================================
    // 2. PARTICLE STRUCT
    // ==========================================
    struct Particle
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double weight = 0.0; // Normalized weight

        // We use a flattened vector for the grid (Row-Major order)
        // Size = MAP_HEIGHT * MAP_WIDTH
        std::vector<float> grid;

        Particle()
        {
            grid.resize(MAP_HEIGHT * MAP_WIDTH, 0.0f);
        }
    };

    // ==========================================
    // 3. MAIN CLASS
    // ==========================================
    class GridFastSlam : public rclcpp::Node
    {
    public:
        GridFastSlam();
        ~GridFastSlam() = default;

        struct RayLUT
        {
            double cos_a;
            double sin_a;
            bool is_valid_fov; // Is this ray within +/- 90 degrees?
        };

    private:
        // --- ROS 2 Interfaces ---
        rclcpp::Subscription<custom_msgs::msg::DeltaOdom>::SharedPtr delta_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        // --- Internal Data ---
        std::vector<Particle> particles_;
        int num_particles_;
        nav_msgs::msg::Path path_msg_;
        int map_update_counter = 0;
        int map_update_interval = 1;
        int current_best_index_ = 0;

        // Random Number Generation
        std::mt19937 rng_;

        // --- Methods (Mapping logic from Python) ---

        // Callback wrappers
        void delta_callback(const custom_msgs::msg::DeltaOdom::SharedPtr msg);
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // Core SLAM Logic (from robot_functions.py)
        void move_particles(double dr1, double dr2, double dt);
        void update_particles(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void update_grid(Particle &p, const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void resample();

        // Helpers
        std::pair<int, int> world_to_grid(double x, double y);
        std::vector<std::pair<int, int>> bresenham(int i0, int j0, int i1, int j1);
        void publish_map_and_path();
        std::vector<std::pair<double, double>> get_scan_endpoints(
            const Particle &p,
            const sensor_msgs::msg::LaserScan::SharedPtr scan,
            const std::vector<RayLUT> &lut,
            bool use_narrow_fov = false // <--- NEW ARGUMENT (Default to false/full view)
        );

        // Math helper
        double normalize_angle(double angle);

        std::vector<RayLUT> scan_lut_;

        // Helper to fill the table
        void init_lut(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    };

} // namespace grid_fastslam

#endif // GRID_FAST_SLAM_HPP_