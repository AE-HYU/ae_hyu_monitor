#include "monitor_node.hpp"

Monitor::Monitor(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options), lap_start_time_(this->get_clock()->now()) {
    
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Parameters - declare and get values
    this->declare_parameter("monitor/loop_rate_hz", 20.0);
    this->declare_parameter("monitor/start_zone_threshold", 1.0);
    this->declare_parameter("monitor/text_size", 14);

    loop_rate_hz_ = this->get_parameter("monitor/loop_rate_hz").as_double();
    start_zone_threshold_ = this->get_parameter("monitor/start_zone_threshold").as_double();
    text_size_ = this->get_parameter("monitor/text_size").as_int();

    RCLCPP_INFO(this->get_logger(), "loop_rate_hz: %f", loop_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "start_zone_threshold: %f", start_zone_threshold_);
    RCLCPP_INFO(this->get_logger(), "text_size: %d", text_size_);

    // Subscribers init
    s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", qos_profile, std::bind(&Monitor::CallbackOdom, this, std::placeholders::_1));
    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile, std::bind(&Monitor::CallbackFrenetOdom, this, std::placeholders::_1));
    s_global_waypoints_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile, std::bind(&Monitor::CallbackGlobalWaypoints, this, std::placeholders::_1));

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
            lap_count_ = -1;  // Indicate invalid start
            RCLCPP_WARN(this->get_logger(), 
                       "Race started from middle of track! Current s: %.2f, max_s: %.2f, threshold: %.2f", 
                       current_s, max_s_, start_zone_threshold_);
        }
        
        previous_s_ = current_s;
        is_first_measurement_ = false;
        return;
    }
    
    // Lap completion detection: crossed from high s to low s
    if (is_race_started_ && previous_s_ > (max_s_ - start_zone_threshold_) && 
        current_s < start_zone_threshold_) {
        
        // Calculate lap time
        auto current_time = this->get_clock()->now();
        double lap_time = (current_time - lap_start_time_).seconds();
        
        // Only record lap time if it's not the first lap (lap 0 -> lap 1)
        if (lap_count_ > 0) {
            lap_times_.push_back(lap_time);
            
            // Update fastest lap time
            if (lap_time < fastest_lap_time_) {
                fastest_lap_time_ = lap_time;
            }
            
            // Calculate mean lap time
            double sum = 0.0;
            for (double time : lap_times_) {
                sum += time;
            }
            mean_lap_time_ = sum / lap_times_.size();
            
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

    
    // Create simple lap info text - just the lap number
    if (!is_race_started_ && lap_count_ == -1) {
        lap_msg.text = "Invalid Start!";
    } else {
        lap_msg.text = "Lap: " + std::to_string(lap_count_);
    }
    
    p_lap_info_->publish(lap_msg);
}

void Monitor::PublishMeanLapTime() {
    rviz_2d_overlay_msgs::msg::OverlayText msg;
    msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    msg.text_size = text_size_;
    
    if (lap_times_.empty()) {
        msg.text = "Mean Lap Time: N/A";
    } else {
        msg.text = "Mean Lap Time: " + std::to_string(mean_lap_time_).substr(0, 5) + "s";
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
    
    // Only track CTE if race is started and not on first lap (lap 0)
    if (is_race_started_ && lap_count_ > 0) {
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
    
    if (!is_race_started_ || lap_count_ <= 0 || total_cte_count_ == 0) {
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
    
    if (!is_race_started_ || lap_count_ <= 0 || max_cte_ == 0.0) {
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
    
    if (!is_race_started_) {
        msg.text = "Current CTE: N/A";
    } else {
        msg.text = "Current CTE: " + std::to_string(current_cte_).substr(0, 5) + "m";
    }
    
    p_current_cte_->publish(msg);
}

int main(int argc, char **argv) {
    std::string node_name = "monitor_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Monitor>(node_name));
    rclcpp::shutdown();
    return 0;
}