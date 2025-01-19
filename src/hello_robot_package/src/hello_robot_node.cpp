#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

enum class RobotState {
    SEARCH,
    FOLLOW
};

class HumanFollowRobot : public rclcpp::Node {
private:
    RobotState current_state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool human_detected_;
    const double SEARCH_ANGULAR_VEL = 1;
    const double FOLLOW_LINEAR_VEL = 1;

    // Image width and height, adjust based on your camera's resolution
    const int IMAGE_WIDTH = 1280;
    const int IMAGE_HEIGHT = 720;
    
    // Define acceptable margins to consider the human "centered"
    const double CENTER_MARGIN_X = 0.1;  // 20% margin of image width
    const double CENTER_MARGIN_Y = 1;  // 20% margin of image height

public:
    HumanFollowRobot() : Node("human_follow_robot"), current_state_(RobotState::SEARCH), human_detected_(false) {
        RCLCPP_INFO(this->get_logger(), "Starting HumanFollowRobot node...");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/human_detections",
            10,
            std::bind(&HumanFollowRobot::detection_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&HumanFollowRobot::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Robot initialized in SEARCH mode");
    }

private:
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        human_detected_ = !msg->detections.empty();
        update_state(msg);
    }

    void update_state(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        RobotState previous_state = current_state_;
        bool centered = false;

        if (human_detected_) {
            // Get the first detection (assuming there is only one human at a time)
            const auto &detection = msg->detections.front();

            // Calculate the center of the bounding box
            double bbox_center_x = (detection.bbox.center.position.x);
            double bbox_center_y = (detection.bbox.center.position.y);

            // Check if the center is within the defined margin of the image center
            double image_center_x = IMAGE_WIDTH / 2.0;
            double image_center_y = IMAGE_HEIGHT / 2.0;

            if (std::abs(bbox_center_x - image_center_x) < (IMAGE_WIDTH * CENTER_MARGIN_X) &&
                std::abs(bbox_center_y - image_center_y) < (IMAGE_HEIGHT * CENTER_MARGIN_Y)) {
                centered = true;
            }
        }

        if (human_detected_ && centered) {
            current_state_ = RobotState::FOLLOW;
        } else {
            current_state_ = RobotState::SEARCH;
        }

        // Log state transitions
        if (previous_state != current_state_) {
            RCLCPP_INFO(this->get_logger(), "State changed to: %s",
                current_state_ == RobotState::SEARCH ? "SEARCH" : "FOLLOW");
        }
    }

    void control_loop() {
        geometry_msgs::msg::Twist msg;

        switch (current_state_) {
            case RobotState::SEARCH:
                msg.angular.z = SEARCH_ANGULAR_VEL;
                msg.linear.x = 0.0;
                break;

            case RobotState::FOLLOW:
                msg.angular.z = 0.0;
                msg.linear.x = FOLLOW_LINEAR_VEL;
                break;
        }

        cmd_vel_pub_->publish(msg);
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollowRobot>());
    rclcpp::shutdown();
    return 0;
}
