#include "monitor_node.hpp"

Monitor::Monitor(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options), lap_start_time_(this->get_clock()->now()) {
    
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Parameters - declare and get values
    this->declare_parameter("monitor/loop_rate_hz", 10.0);
    this->declare_parameter("monitor/start_zone_threshold", 1.0);
    this->declare_parameter("monitor/text_size", 14);
    this->declare_parameter("monitor/odom_topic", "/odom");

    loop_rate_hz_ = this->get_parameter("monitor/loop_rate_hz").as_double();
    start_zone_threshold_ = this->get_parameter("monitor/start_zone_threshold").as_double();
    text_size_ = this->get_parameter("monitor/text_size").as_int();
    std::string odom_topic = this->get_parameter("monitor/odom_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", loop_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "start_zone_threshold: %f", start_zone_threshold_);
    RCLCPP_INFO(this->get_logger(), "text_size: %d", text_size_);
    RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic.c_str());

    // Subscribers init
    s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_profile, std::bind(&Monitor::CallbackOdom, this, std::placeholders::_1));
    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile, std::bind(&Monitor::CallbackFrenetOdom, this, std::placeholders::_1));
    s_global_waypoints_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile, std::bind(&Monitor::CallbackGlobalWaypoints, this, std::placeholders::_1));
    s_obstacles_ = this->create_subscription<ae_hyu_msgs::msg::ObstacleArray>(
        "/tracking/obstacles", qos_profile, std::bind(&Monitor::CallbackObstacles, this, std::placeholders::_1));

    // Publisher init
    p_lap_info_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/lap_info", qos_profile);
    p_current_speed_info_ = this->create_publisher<std_msgs::msg::Float32>(
        "/monitor/current_speed", qos_profile);
    p_mean_lap_time_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/mean_lap_time", qos_profile);
    p_fastest_lap_time_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/fastest_lap_time", qos_profile);
    p_mean_cte_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/mean_cte", qos_profile);
    p_max_cte_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/max_cte", qos_profile);
    p_current_cte_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/current_cte", qos_profile);
    p_obstacle_info_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/monitor/obstacle_info", qos_profile);
        
    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });
        
    RCLCPP_INFO(this->get_logger(), "Monitor node initialized successfully");
}

Monitor::~Monitor() {}

void Monitor::Run() {
    auto current_time = this->get_clock()->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Monitor Running ...");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (b_is_odom_ == false) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Waiting for odom...");
        return;
    }
    if (b_is_frenet_odom_ == false) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Waiting for frenet odom...");
        return;
    }
    // Track length is optional for monitoring, so no need to wait for it

    nav_msgs::msg::Odometry odom; {
        std::lock_guard<std::mutex> lock(mutex_odom_);
        odom = i_current_odom_;
    }
    nav_msgs::msg::Odometry frenet_odom; {
        std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
        frenet_odom = i_current_frenet_odom_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Process & Publish monitor information
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // Update and publish lap information
    UpdateLapInfo(frenet_odom);
    PublishLapInfo();

    // Update CTE information
    UpdateCTE(frenet_odom);

    // Publish lap time information
    PublishMeanLapTime();
    PublishFastestLapTime();

    // Publish CTE information
    PublishMeanCTE();
    PublishMaxCTE();
    PublishCurrentCTE();

    // Publish obstacle information
    PublishObstacleInfo();

    // Publish speed information
    PublishSpeedInfo(odom);
}

void Monitor::PublishSpeedInfo(const nav_msgs::msg::Odometry& odom) {
    // Calculate current speed from odometry
    double linear_velocity = odom.twist.twist.linear.x;
    
    // Publish speed in m/s for PieChart visualization
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(linear_velocity);
    p_current_speed_info_->publish(speed_msg);
}

void Monitor::UpdateLapInfo(const nav_msgs::msg::Odometry& frenet_odom) {
    // Get current s coordinate
    double current_s = frenet_odom.pose.pose.position.x;
    
    // Wait until we have max_s information
    if (max_s_ <= 0.0) {
        return;
    }
    
    // First measurement - check if we're starting from proper position
    if (is_first_measurement_) {
        // Check if starting position is in the start zone
        bool in_start_zone = (current_s < start_zone_threshold_) || 
                             (current_s > (max_s_ - start_zone_threshold_));
        
        if (in_start_zone) {
            is_race_started_ = true;
            lap_count_ = 0;  // Start from lap 0
            lap_start_time_ = this->get_clock()->now();  // Start timing
            RCLCPP_INFO(this->get_logger(), 
                       "Race started from proper position. Current s: %.2f, threshold: %.2f", 
                       current_s, start_zone_threshold_);
        } else {
            is_race_started_ = false;
            lap_count_ = 0;  // Start counting from 0 even for invalid start
            lap_start_time_ = this->get_clock()->now();  // Start timing even for invalid start
            RCLCPP_WARN(this->get_logger(), 
                       "Race started from middle of track! Current s: %.2f, max_s: %.2f, threshold: %.2f", 
                       current_s, max_s_, start_zone_threshold_);
            RCLCPP_WARN(this->get_logger(), 
                       "Lap counting and timing enabled from invalid start position!");
        }
        
        previous_s_ = current_s;
        is_first_measurement_ = false;
        return;
    }
    
    // Lap completion detection: crossed from high s to low s
    if (previous_s_ > (max_s_ - start_zone_threshold_) && 
        current_s < start_zone_threshold_) {
        
        // Calculate lap time
        auto current_time = this->get_clock()->now();
        double lap_time = (current_time - lap_start_time_).seconds();
        
        // Record lap time if it's not the first lap (regardless of start validity)
        if (lap_count_ > 0) {
            lap_times_.push_back(lap_time);
            previous_lap_time_ = lap_time;  // Store previous lap time
            
            // Update fastest lap time
            if (lap_time < fastest_lap_time_) {
                fastest_lap_time_ = lap_time;
            }
            
            // Calculate mean lap time for recent 5 laps
            size_t recent_laps = std::min(static_cast<size_t>(5), lap_times_.size());
            double sum = 0.0;
            for (size_t i = lap_times_.size() - recent_laps; i < lap_times_.size(); i++) {
                sum += lap_times_[i];
            }
            mean_lap_time_ = sum / recent_laps;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Lap %d completed! Time: %.2f s, Fastest: %.2f s, Mean: %.2f s, Mean CTE: %.3f m", 
                       lap_count_, lap_time, fastest_lap_time_, mean_lap_time_, mean_cte_);
        }
        
        lap_count_++;
        lap_start_time_ = current_time;  // Reset lap start time for next lap
        
        // Reset CTE tracking for new lap
        current_lap_cte_sum_ = 0.0;
        current_lap_cte_count_ = 0;
        mean_cte_ = 0.0;
        
        // Reset mean CTE every 10 laps
        if (lap_count_ % 10 == 0) {
            total_cte_sum_ = 0.0;
            total_cte_count_ = 0;
            ten_lap_mean_cte_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "10-lap mean CTE reset at lap %d", lap_count_);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Starting lap %d. s: %.2f -> %.2f", lap_count_, previous_s_, current_s);
    }
    
    // Update previous s for next iteration
    previous_s_ = current_s;
}

void Monitor::PublishLapInfo() {
    rviz_2d_overlay_msgs::msg::OverlayText lap_msg;
    lap_msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    lap_msg.text_size = text_size_;  // Use parameter value

    // Calculate current lap time (time since lap started)
    auto current_time = this->get_clock()->now();
    double current_lap_time = (current_time - lap_start_time_).seconds();
    
    // Create lap info text with current lap number, current lap time, and previous lap time
    std::string lap_text = "Lap: " + std::to_string(lap_count_) + 
                          " | Lap Time: " + std::to_string(current_lap_time).substr(0, 5) + "s";
    
    if (previous_lap_time_ > 0.0) {
        lap_text += " | Prev. Lap Time: " + std::to_string(previous_lap_time_).substr(0, 5) + "s";
    }
    
    lap_msg.text = lap_text;
    
    p_lap_info_->publish(lap_msg);
}

void Monitor::PublishMeanLapTime() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    if (lap_times_.empty()) {
        msg.text = "Mean Lap Time (5lap): N/A";
    } else {
        msg.text = "Mean Lap Time (5lap): " + std::to_string(mean_lap_time_).substr(0, 5) + "s";
    }
    
    p_mean_lap_time_->publish(msg);
}

void Monitor::PublishFastestLapTime() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    if (fastest_lap_time_ == std::numeric_limits<double>::max()) {
        msg.text = "Fastest Lap Time: N/A";
    } else {
        msg.text = "Fastest Lap Time: " + std::to_string(fastest_lap_time_).substr(0, 5) + "s";
    }
    
    p_fastest_lap_time_->publish(msg);
}

void Monitor::UpdateCTE(const nav_msgs::msg::Odometry& frenet_odom) {
    // Get current Cross Track Error (frenet d coordinate)
    current_cte_ = std::abs(frenet_odom.pose.pose.position.y);
    
    // Track CTE if we have valid data (lap_count > 0)
    if (lap_count_ > 0) {
        // Update current lap CTE statistics
        current_lap_cte_sum_ += current_cte_;
        current_lap_cte_count_++;
        mean_cte_ = current_lap_cte_sum_ / current_lap_cte_count_;
        
        // Update 10-lap rolling mean CTE
        total_cte_sum_ += current_cte_;
        total_cte_count_++;
        ten_lap_mean_cte_ = total_cte_sum_ / total_cte_count_;
        
        // Update maximum CTE across all laps
        if (current_cte_ > max_cte_) {
            max_cte_ = current_cte_;
        }
    }
}

void Monitor::PublishMeanCTE() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    if (total_cte_count_ == 0) {
        msg.text = "Mean CTE (10lap): N/A";
    } else {
        msg.text = "Mean CTE (10lap): " + std::to_string(ten_lap_mean_cte_).substr(0, 5) + "m";
    }
    
    p_mean_cte_->publish(msg);
}

void Monitor::PublishMaxCTE() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    if (max_cte_ == 0.0) {
        msg.text = "Max CTE: N/A";
    } else {
        msg.text = "Max CTE: " + std::to_string(max_cte_).substr(0, 5) + "m";
    }
    
    p_max_cte_->publish(msg);
}

void Monitor::PublishCurrentCTE() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    // Show current CTE always (even for invalid starts)
    msg.text = "Current CTE: " + std::to_string(current_cte_).substr(0, 5) + "m";
    
    p_current_cte_->publish(msg);
}

void Monitor::PublishObstacleInfo() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    std::string obstacle_info_text = "";
    
    bool is_static_detected = false;
    bool is_dynamic_detected = false;
    
    // Check if we have obstacle data
    if (b_is_obstacles_) {
        std::lock_guard<std::mutex> lock(mutex_obstacles_);
        
        // Analyze obstacles to find static and dynamic ones
        for (const auto& obstacle : i_current_obstacles_.obstacles) {
            if (obstacle.is_static) {
                is_static_detected = true;
            } else {
                is_dynamic_detected = true;
            }
        }
    }
    
    // Create obstacle info text with line breaks
    if (!b_is_obstacles_) {
        obstacle_info_text += "Static: N/A\nDynamic: N/A";
    } else {
        obstacle_info_text += "Static: " + std::string(is_static_detected ? "True" : "False") + 
                             "\nDynamic: " + std::string(is_dynamic_detected ? "True" : "False");
    }
    
    msg.text = obstacle_info_text;
    p_obstacle_info_->publish(msg);
}

int main(int argc, char **argv) {
    std::string node_name = "monitor_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Monitor>(node_name));
    rclcpp::shutdown();
    return 0;
}