#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class JaraSimMarker : public rclcpp::Node
{
public:
    JaraSimMarker()
        : Node("jara_sim_marker")
    {
        // ❌ use_sim_time 은 declare 하지 않는다 (ROS2가 자동 관리)

        declare_parameter("frame_id", std::string("base_link"));
        frame_id_ = get_parameter("frame_id").as_string();

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/jara_robot_markers", 10);

        // 라이다(원기둥) 파라미터
        declare_parameter("lidar_x", 0.0);
        declare_parameter("lidar_y", 0.0);
        declare_parameter("lidar_z", 0.12);
        declare_parameter("lidar_d", 0.08);
        declare_parameter("lidar_h", 0.05);

        timer_ = create_wall_timer(
            100ms, std::bind(&JaraSimMarker::publishMarkers, this));

        RCLCPP_INFO(
            get_logger(),
            "JaraSimMarker started (frame_id=%s) topic=/jara_robot_markers (MarkerArray)",
            frame_id_.c_str());
    }

private:
    void publishMarkers()
    {
        // ✅ RViz 깜박임/사라짐 방지 핵심
        const auto stamp0 = rclcpp::Time(0, 0, RCL_ROS_TIME);

        visualization_msgs::msg::MarkerArray arr;

        // --------------------
        // 1) Body
        // --------------------
        visualization_msgs::msg::Marker body;
        body.header.stamp = stamp0;
        body.header.frame_id = frame_id_;
        body.frame_locked = true;

        body.ns = "jarabot";
        body.id = 0;
        body.type = visualization_msgs::msg::Marker::CUBE;
        body.action = visualization_msgs::msg::Marker::ADD;

        body.pose.position.x = 0.0;
        body.pose.position.y = 0.0;
        body.pose.position.z = 0.05;
        body.pose.orientation.w = 1.0;

        body.scale.x = 0.40;
        body.scale.y = 0.20;
        body.scale.z = 0.10;

        body.color.b = 1.0f;
        body.color.a = 1.0f;

        body.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(body);

        // --------------------
        // 2) Arrow
        // --------------------
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = stamp0;
        arrow.header.frame_id = frame_id_;
        arrow.frame_locked = true;

        arrow.ns = "jarabot";
        arrow.id = 1;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.pose.position.x = body.scale.x * 0.25;
        arrow.pose.position.z = body.scale.z * 0.6;
        arrow.pose.orientation.w = 1.0;

        arrow.scale.x = body.scale.x * 0.8;
        arrow.scale.y = 0.03;
        arrow.scale.z = 0.03;

        arrow.color.r = 1.0f;
        arrow.color.a = 1.0f;

        arrow.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(arrow);

        // --------------------
        // 3) Lidar (Yellow Cylinder)
        // --------------------
        visualization_msgs::msg::Marker lidar;
        lidar.header.stamp = stamp0;
        lidar.header.frame_id = frame_id_;
        lidar.frame_locked = true;

        lidar.ns = "jarabot";
        lidar.id = 2;
        lidar.type = visualization_msgs::msg::Marker::CYLINDER;
        lidar.action = visualization_msgs::msg::Marker::ADD;

        lidar.pose.position.x = get_parameter("lidar_x").as_double();
        lidar.pose.position.y = get_parameter("lidar_y").as_double();
        lidar.pose.position.z = get_parameter("lidar_z").as_double();
        lidar.pose.orientation.w = 1.0;

        lidar.scale.x = get_parameter("lidar_d").as_double();
        lidar.scale.y = get_parameter("lidar_d").as_double();
        lidar.scale.z = get_parameter("lidar_h").as_double();

        lidar.color.r = 1.0f;
        lidar.color.g = 1.0f;
        lidar.color.a = 1.0f;

        lidar.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(lidar);

        marker_pub_->publish(arr);
    }

    std::string frame_id_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimMarker>());
    rclcpp::shutdown();
    return 0;
}
