#include <omp.h>
#include "grid_fastslam/grid_fastslam.hpp"

namespace grid_fastslam
{

    GridFastSlam::GridFastSlam()
        : Node("grid_fast_slam_node"),
          rng_(std::random_device{}())
    {
        this->declare_parameter("num_particles", 10);
        num_particles_ = this->get_parameter("num_particles").as_int();

        particles_.resize(num_particles_);

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.reliable();
        map_qos.transient_local();

        delta_sub_ = this->create_subscription<custom_msgs::msg::DeltaOdom>(
            "/delta", 10,
            std::bind(&GridFastSlam::delta_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GridFastSlam::scan_callback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/particle_robot_path", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/particle_pose", 10);
        particles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/particles", 10);

        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "GridFastSlam initialized with %d particles.", num_particles_);
    }

    void GridFastSlam::delta_callback(const custom_msgs::msg::DeltaOdom::SharedPtr msg)
    {
        double dr1 = msg->dr1;
        double dr2 = msg->dr2;
        double dt = msg->dt;

        if (std::abs(dr1) < 1e-6 && std::abs(dr2) < 1e-6 && std::abs(dt) < 1e-6)
        {
            return;
        }

        this->move_particles(dr1, dr2, dt);
    }

    void GridFastSlam::move_particles(double dr1, double dr2, double dt)
    {
        const double alpha1 = 0.1;
        const double alpha2 = 0.1;
        const double alpha3 = 0.01;
        const double alpha4 = 0.01;

        double sigma_rot1 = alpha1 * std::abs(dr1) + alpha2 * dt;
        double sigma_trans = alpha3 * dt + alpha4 * (std::abs(dr1) + std::abs(dr2));
        double sigma_rot2 = alpha1 * std::abs(dr2) + alpha2 * dt;

        std::normal_distribution<double> noise_rot1(0.0, sigma_rot1);
        std::normal_distribution<double> noise_trans(0.0, sigma_trans);
        std::normal_distribution<double> noise_rot2(0.0, sigma_rot2);

        for (auto &p : particles_)
        {
            double rot1_hat = dr1 + noise_rot1(rng_);
            double trans_hat = dt + noise_trans(rng_);
            double rot2_hat = dr2 + noise_rot2(rng_);

            p.x += trans_hat * std::cos(p.yaw + rot1_hat);
            p.y += trans_hat * std::sin(p.yaw + rot1_hat);
            p.yaw += rot1_hat + rot2_hat;

            p.yaw = normalize_angle(p.yaw);
        }
    }

    double GridFastSlam::normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // =========================================================
    // HELPER FUNCTIONS (COORDINATES & RAYCASTING)
    // =========================================================

    std::pair<int, int> GridFastSlam::world_to_grid(double x, double y)
    {
        // Convert World (meters) to Map (indices)
        // Python: j = floor((x - ox) / res), i = floor((y - oy) / res)
        int j = std::floor((x - OX) / RESOLUTION); // x is column (width)
        int i = std::floor((y - OY) / RESOLUTION); // y is row (height)
        return {i, j};
    }

    std::vector<std::pair<int, int>> GridFastSlam::bresenham(int i0, int j0, int i1, int j1)
    {
        // The "Line Drawing" Algorithm
        // This connects two grid cells with the pixels in between
        std::vector<std::pair<int, int>> cells;
        cells.reserve(500);

        int di = std::abs(i1 - i0);
        int dj = std::abs(j1 - j0);
        int si = (i0 < i1) ? 1 : -1;
        int sj = (j0 < j1) ? 1 : -1;
        int err = di - dj;

        int i = i0;
        int j = j0;

        while (true)
        {
            cells.push_back({i, j});
            if (i == i1 && j == j1)
                break;
            int e2 = 2 * err;
            if (e2 > -dj)
            {
                err -= dj;
                i += si;
            }
            if (e2 < di)
            {
                err += di;
                j += sj;
            }
        }
        return cells;
    }

    void GridFastSlam::init_lut(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Only calculate if the table is empty or size changed
        if (scan_lut_.size() == scan->ranges.size())
            return;

        scan_lut_.clear();
        scan_lut_.reserve(scan->ranges.size());

        double angle = scan->angle_min;

        // Define Field of View Limits (-PI/2 to +PI/2)
        const double angle_min_limit = -M_PI_4;
        const double angle_max_limit = M_PI_4;

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            RayLUT entry;

            // 1. Normalize Angle (The heavy "while" loop logic)
            double angle_wrapped = angle;
            while (angle_wrapped > M_PI)
                angle_wrapped -= 2.0 * M_PI;
            while (angle_wrapped < -M_PI)
                angle_wrapped += 2.0 * M_PI;

            // 2. Pre-calculate Trigonometry
            entry.cos_a = std::cos(angle); // Note: cos(angle) == cos(wrapped)
            entry.sin_a = std::sin(angle);

            // 3. Pre-calculate FOV check
            if (angle_wrapped >= angle_min_limit && angle_wrapped <= angle_max_limit)
            {
                entry.is_valid_fov = true;
            }
            else
            {
                entry.is_valid_fov = false;
            }

            scan_lut_.push_back(entry);

            angle += scan->angle_increment;
        }

        RCLCPP_INFO(this->get_logger(), "LUT Initialized with %zu rays.", scan_lut_.size());
    }

    // =========================================================
    // GRID UPDATE LOGIC
    // =========================================================

    // Internal helper to calculate where laser points land in the world
    std::vector<std::pair<double, double>> GridFastSlam::get_scan_endpoints(
        const Particle &p,
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const std::vector<GridFastSlam::RayLUT> &lut,
        bool use_narrow_fov)
    {
        std::vector<std::pair<double, double>> points;
        points.reserve(scan->ranges.size());

        double limit_max = scan->range_max - 0.001;

        // Pre-compute particle rotation
        double sin_p = std::sin(p.yaw);
        double cos_p = std::cos(p.yaw);

        for (size_t k = 0; k < scan->ranges.size(); ++k)
        {

            if (!use_narrow_fov || lut[k].is_valid_fov)
            {
                double r = scan->ranges[k];

                if (std::isfinite(r) && r > scan->range_min && r < limit_max)
                {
                    double lx = r * lut[k].cos_a;
                    double ly = r * lut[k].sin_a;

                    double wx = (cos_p * lx - sin_p * ly) + p.x;
                    double wy = (sin_p * lx + cos_p * ly) + p.y;
                    points.push_back({wx, wy});
                }
            }
        }
        return points;
    }

    void GridFastSlam::update_grid(Particle &p, const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // 1. Get where the laser hits are in the real world
        auto endpoints = get_scan_endpoints(p, scan, scan_lut_, true);
        if (endpoints.empty())
            return;

        // 2. Get Robot's position in the grid
        auto [r0, c0] = world_to_grid(p.x, p.y);

        // Sanity check: if robot is off-map, we can't raycast from it
        if (r0 < 0 || r0 >= MAP_HEIGHT || c0 < 0 || c0 >= MAP_WIDTH)
            return;

        // 3. Loop through rays
        int beam_step = 1; 

        for (size_t k = 0; k < endpoints.size(); k += beam_step)
        {
            auto [wx, wy] = endpoints[k];
            auto [r1, c1] = world_to_grid(wx, wy);

            // Check if endpoint is inside map
            if (r1 < 0 || r1 >= MAP_HEIGHT || c1 < 0 || c1 >= MAP_WIDTH)
                continue;

            // 4. Raycast (Bresenham) to update free space
            auto ray = bresenham(r0, c0, r1, c1);

            // Iterate through the ray (excluding the last point, which is the hit)
            for (size_t j = 0; j < ray.size() - 1; ++j)
            {
                int fr = ray[j].first;
                int fc = ray[j].second;

                // Boundary check for ray points (just in case bresenham goes slightly out)
                if (fr >= 0 && fr < MAP_HEIGHT && fc >= 0 && fc < MAP_WIDTH)
                {
                    int idx = fr * MAP_WIDTH + fc; // 2D -> 1D Index
                    p.grid[idx] += L_FREE;
                    
                    if (p.grid[idx] < L_MIN)
                        p.grid[idx] = L_MIN; // Clamp
                }
            }

            // 5. Update the endpoint as occupied
            int idx_hit = r1 * MAP_WIDTH + c1;
            p.grid[idx_hit] += L_OCC;
            
            if (p.grid[idx_hit] > L_MAX)
                p.grid[idx_hit] = L_MAX; // Clamp
        }
    }

    // =========================================================
    // PROBABILITY & RESAMPLING LOGIC
    // =========================================================

    // Helper: Numerical stability for normalizing weights
    // Computes log(sum(exp(x_i))) without overflowing
    double log_sum_exp(const std::vector<double> &log_weights)
    {
        if (log_weights.empty())
            return -std::numeric_limits<double>::infinity();

        // 1. Find the maximum value to shift the range
        double max_log_w = *std::max_element(log_weights.begin(), log_weights.end());

        // 2. Compute sum of exponentials of the differences
        double sum = 0.0;
        for (double lw : log_weights)
        {
            sum += std::exp(lw - max_log_w);
        }

        // 3. Return result in log domain
        return max_log_w + std::log(sum);
    }

    void GridFastSlam::update_particles(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::vector<double> log_weights(num_particles_);

// 1. Calculate weight for EACH particle
#pragma omp parallel for
        for (int i = 0; i < num_particles_; ++i)
        {
            const auto &p = particles_[i];

            // Get valid scan points in World Frame
            auto endpoints = get_scan_endpoints(p, scan, scan_lut_, false);

            // If no valid points, this particle is dead
            if (endpoints.empty())
            {
                log_weights[i] = -1.0e9;
                continue;
            }

            double log_w_sum = 0.0;
            int hit_count = 0;

            // Compare scan to the particle's OWN map
            for (const auto &pt : endpoints)
            {
                auto [r, c] = world_to_grid(pt.first, pt.second);

                // Check if point is inside the map
                if (r >= 0 && r < MAP_HEIGHT && c >= 0 && c < MAP_WIDTH)
                {
                    float log_odds = p.grid[r * MAP_WIDTH + c];

                    double prob = 1.0 / (1.0 + std::exp(-log_odds));
                    prob = std::max(1e-6, std::min(prob, 1.0 - 1e-6));

                    log_w_sum += std::log(prob);
                    hit_count++;
                }
            }

            if (hit_count > 0)
                log_weights[i] = log_w_sum / hit_count;
            else
                log_weights[i] = -1.0e9;
        }

        bool any_valid_weight = false;
        for (double w : log_weights)
        {
            if (w > -1.0e8)
            {
                any_valid_weight = true;
                break;
            }
        }

        // 2. Normalize Weights (LogSumExp Trick)
        if (!any_valid_weight)
        {
            for (auto &p : particles_)
                p.weight = 1.0 / num_particles_;
        }
        else
        {
            double log_sum = log_sum_exp(log_weights);
            for (int i = 0; i < num_particles_; ++i)
            {
                particles_[i].weight = std::exp(log_weights[i] - log_sum);
            }
        }
    }

    void GridFastSlam::resample()
    {
        // 1. Calculate Effective Sample Size (N_eff)
        double sum_sq = 0.0;
        for (const auto &p : particles_)
        {
            sum_sq += (p.weight * p.weight);
        }
         double n_eff = 1.0 / (sum_sq + 1e-9);

        // 2. Resample only if particles have degraded
        if (n_eff <= num_particles_ * 0.9)
        {

            std::vector<Particle> new_particles;
            new_particles.reserve(num_particles_);

            std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
            double r = dist(rng_);           // Random start point
            double c = particles_[0].weight; // Cumulative weight
            int i = 0;

            for (int m = 0; m < num_particles_; ++m)
            {
                double u = r + (double)m / num_particles_; // Step through the wheel

                // Find the particle that spans this weight segment
                while (u > c && i < num_particles_ - 1)
                {
                    i++;
                    c += particles_[i].weight;
                }

                // Copy the particle
                Particle p_copy = particles_[i];
                p_copy.weight = 1.0 / num_particles_; // Reset weight after resampling
                new_particles.push_back(p_copy);
            }

            // Replace old set with new set
            particles_ = new_particles;
            current_best_index_ = 0;
        }
    }

    void GridFastSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now(); 
        // Initialize Lookup Table if needed
        init_lut(msg);

        // 1. Update Weights based on how well scan matches map
        update_particles(msg);

        // 2. Resample (Evolution) - Kill bad particles, multiply good ones
        resample();

        // 3. Update Maps (Grid) for each particle
        map_update_counter++;
        if (map_update_counter >= map_update_interval)
        {
#pragma omp parallel for
            for (auto &p : particles_)
            {
                update_grid(p, msg);
            }
            map_update_counter = 0;
        }

        // 4. Publish Visualization
        publish_map_and_path();

        auto end_time = std::chrono::high_resolution_clock::now();

        // 3. Calcular la duración en milisegundos (puedes cambiar a microseconds si es muy rápido)
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // 4. Imprimir en consola (Usando el logger de ROS 2)
        // Usamos RCLCPP_INFO_STREAM para facilidad, o RCLCPP_INFO con formato
        RCLCPP_INFO(this->get_logger(), "Scan Callback tardó: %ld ms", duration);
    }

    void GridFastSlam::publish_map_and_path()
    {
        // Find the single best particle to visualize
        auto best_it = std::max_element(particles_.begin(), particles_.end(),
                                        [](const Particle &a, const Particle &b)
                                        { return a.weight < b.weight; });

        int actual_best_index = std::distance(particles_.begin(), best_it);
        double best_weight = particles_[actual_best_index].weight;

        double current_display_weight = particles_[current_best_index_].weight;
        if (best_weight > (current_display_weight * 1.20))
        {
            current_best_index_ = actual_best_index;
        }
        if (current_best_index_ >= num_particles_)
            current_best_index_ = 0;

        const auto &display_p = particles_[current_best_index_];

        // --- Publish Map ---
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";
        map_msg.info.resolution = RESOLUTION;
        map_msg.info.width = MAP_WIDTH;
        map_msg.info.height = MAP_HEIGHT;
        map_msg.info.origin.position.x = OX;
        map_msg.info.origin.position.y = OY;

        // Orientation must be valid quaternion (0,0,0,1)
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data.resize(MAP_WIDTH * MAP_HEIGHT);

        // Convert Log-Odds to Int8 [0-100] for RViz
        for (size_t i = 0; i < display_p.grid.size(); ++i)
        {
            // Probability p = 1 / (1 + exp(-l))
            double p = 1.0 / (1.0 + std::exp(-display_p.grid[i]));
            map_msg.data[i] = static_cast<int8_t>(p * 100);
        }
        map_pub_->publish(map_msg);

        // --- Publish Path ---
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map_msg.header;
        pose.pose.position.x = display_p.x;
        pose.pose.position.y = display_p.y;


        // Yaw to Quaternion (Simple Z-axis rotation)
        pose.pose.orientation.z = std::sin(display_p.yaw * 0.5);
        pose.pose.orientation.w = std::cos(display_p.yaw * 0.5);

        path_msg_.header.stamp = this->now();
        path_msg_.poses.push_back(pose);

        pose_pub_->publish(pose);
        path_pub_->publish(path_msg_);

        // --- Publish Particles as PointCloud2 ---
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map"; // Same frame as your grid
        cloud_msg.height = 1; 
        cloud_msg.width = particles_.size();

        // Initialize the PointCloud fields (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(particles_.size());

        // Create iterators to write data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto &p : particles_)
        {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0; // Flat on the map

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        particles_pub_->publish(cloud_msg);
    }

} // namespace grid_fastslam