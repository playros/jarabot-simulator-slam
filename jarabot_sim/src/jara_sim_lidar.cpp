#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <limits>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

struct Segment
{
    double x1, y1, x2, y2;
};

class JaraSimLidar : public rclcpp::Node
{
public:
    JaraSimLidar()
        : Node("jara_sim_lidar"),
          angle_min_(-3.0 * M_PI / 4.0),
          angle_max_(3.0 * M_PI / 4.0),
          angle_increment_(0.0058),
          range_min_(0.05),
          range_max_(8.0),
          publish_rate_hz_(10.0),
          range_noise_std_(0.01),
          has_odom_(false),
          room_half_(3.0),             // NEW
          inner_span_ratio_(0.5),       // NEW
          inner_open_side_("+x"),       // NEW
          rng_(std::random_device{}()),
          noise_dist_(0.0, 1.0)
    {
        // 기존 파라미터
        declare_parameter("angle_min", angle_min_);
        declare_parameter("angle_max", angle_max_);
        declare_parameter("angle_increment", angle_increment_);
        declare_parameter("range_min", range_min_);
        declare_parameter("range_max", range_max_);
        declare_parameter("publish_rate_hz", publish_rate_hz_);
        declare_parameter("range_noise_std", range_noise_std_);

        // NEW: 맵(벽) 파라미터
        declare_parameter("room_half", room_half_);                 // 외벽 반폭 (기본 3.0 => [-3,3])
        declare_parameter("inner_span_ratio", inner_span_ratio_);   // 내부 ㄷ자 스팬 비율 (기본 0.5)
        declare_parameter("inner_open_side", inner_open_side_);     // "+x", "-x", "+y", "-y"

        // 파라미터 로드
        angle_min_        = get_parameter("angle_min").as_double();
        angle_max_        = get_parameter("angle_max").as_double();
        angle_increment_  = get_parameter("angle_increment").as_double();
        range_min_        = get_parameter("range_min").as_double();
        range_max_        = get_parameter("range_max").as_double();
        publish_rate_hz_  = get_parameter("publish_rate_hz").as_double();
        range_noise_std_  = get_parameter("range_noise_std").as_double();

        room_half_        = get_parameter("room_half").as_double();
        inner_span_ratio_ = get_parameter("inner_span_ratio").as_double();
        inner_open_side_  = get_parameter("inner_open_side").as_string();

        // Odom 구독
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&JaraSimLidar::odomCallback, this, std::placeholders::_1));

        // Scan 퍼블리시
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // 타이머
        auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JaraSimLidar::publishScan, this));

        // 벽(segments_) 생성
        buildSegments();

        const double room_size = room_half_ * 2.0;
        RCLCPP_INFO(get_logger(),
            "JaraSimLidar started. Outer room half=%.2f (size=%.2f). Inner span ratio=%.2f, open=%s",
            room_half_, room_size, inner_span_ratio_, inner_open_side_.c_str());
    }

private:
    void buildSegments()
    {
        segments_.clear();

        const double h = room_half_;       // 외벽 반폭
        const double L = h * 2.0;          // 외벽 한 변 길이

        // 1) 외벽 사각형
        segments_.push_back({-h, -h, -h,  h});
        segments_.push_back({ h, -h,  h,  h});
        segments_.push_back({-h, -h,  h, -h});
        segments_.push_back({-h,  h,  h,  h});

        // 2) 내부 ㄷ자: 스팬 = L * inner_span_ratio (기본 L/2)
        double inner_span = L * inner_span_ratio_;
        if (inner_span <= 0.0) inner_span = L * 0.5;
        if (inner_span > L) inner_span = L; // 안전 클램프

        const double ih = inner_span / 2.0; // 내부 반스팬
        const double xmin = -ih, xmax = ih;
        const double ymin = -ih, ymax = ih;

        // inner_open_side_ : "+x"면 오른쪽 면을 비움
        if (inner_open_side_ == "+x") {
            // left, top, bottom
            segments_.push_back({xmin, ymin, xmin, ymax}); // left
            segments_.push_back({xmin, ymax, xmax, ymax}); // top
            segments_.push_back({xmin, ymin, xmax, ymin}); // bottom
        } else if (inner_open_side_ == "-x") {
            // right, top, bottom
            segments_.push_back({xmax, ymin, xmax, ymax}); // right
            segments_.push_back({xmin, ymax, xmax, ymax}); // top
            segments_.push_back({xmin, ymin, xmax, ymin}); // bottom
        } else if (inner_open_side_ == "+y") {
            // bottom, left, right
            segments_.push_back({xmin, ymin, xmax, ymin}); // bottom
            segments_.push_back({xmin, ymin, xmin, ymax}); // left
            segments_.push_back({xmax, ymin, xmax, ymax}); // right
        } else if (inner_open_side_ == "-y") {
            // top, left, right
            segments_.push_back({xmin, ymax, xmax, ymax}); // top
            segments_.push_back({xmin, ymin, xmin, ymax}); // left
            segments_.push_back({xmax, ymin, xmax, ymax}); // right
        } else {
            // 잘못된 값이면 기본(+x)
            segments_.push_back({xmin, ymin, xmin, ymax}); // left
            segments_.push_back({xmin, ymax, xmax, ymax}); // top
            segments_.push_back({xmin, ymin, xmax, ymin}); // bottom
            RCLCPP_WARN(get_logger(), "inner_open_side invalid. Using default +x");
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        const auto & q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        theta_ = std::atan2(siny_cosp, cosy_cosp);

        has_odom_ = true;
    }

    double raycastSegment(double dx, double dy, const Segment & seg)
    {
        // Ray: origin(0,0) + s*(dx,dy), s>=0
        // Segment: (x1,y1) + t*(sx,sy), t in [0,1]
        double sx = seg.x2 - seg.x1;
        double sy = seg.y2 - seg.y1;

        double det = (-dx * sy + dy * sx);
        if (std::fabs(det) < 1e-9) {
            return std::numeric_limits<double>::infinity();
        }

        // NOTE: This formulation assumes seg is already in robot frame (origin at robot).
        double s = (-sy * (seg.x1) + sx * (seg.y1)) / det;
        double t = ( dx * (seg.y1) - dy * (seg.x1)) / det;

        if (s >= 0.0 && t >= 0.0 && t <= 1.0) {
            return s;
        }
        return std::numeric_limits<double>::infinity();
    }

    void publishScan()
    {
        if (!has_odom_) return;

        auto now_time = now();

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now_time;
        scan.header.frame_id = "base_link";

        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.range_min = range_min_;
        scan.range_max = range_max_;
        scan.scan_time = 1.0 / publish_rate_hz_;
        scan.time_increment = 0.0;

        int beam_count = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;
        scan.ranges.resize(beam_count);

        for (int i = 0; i < beam_count; ++i) {
            double angle = angle_min_ + i * angle_increment_;
            double global_angle = theta_ + angle;

            double dx = std::cos(global_angle);
            double dy = std::sin(global_angle);

            double min_t = std::numeric_limits<double>::infinity();

            for (const auto & seg_world : segments_) {
                // 월드(고정) 벽을 로봇 기준 좌표로 이동(평행이동만; 회전은 광선이 global_angle이므로 OK)
                Segment seg_robot;
                seg_robot.x1 = seg_world.x1 - x_;
                seg_robot.y1 = seg_world.y1 - y_;
                seg_robot.x2 = seg_world.x2 - x_;
                seg_robot.y2 = seg_world.y2 - y_;

                double t = raycastSegment(dx, dy, seg_robot);
                if (t < min_t) min_t = t;
            }

            double range = range_max_;
            if (std::isfinite(min_t)) range = min_t;

            // min/max clamp
            if (range < range_min_) range = range_min_;
            if (range > range_max_) range = range_max_;

            // noise
            range += noise_dist_(rng_) * range_noise_std_;
            if (range < range_min_) range = range_min_;
            if (range > range_max_) range = range_max_;

            scan.ranges[i] = static_cast<float>(range);
        }

        scan_pub_->publish(scan);
    }

    // laser params
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double publish_rate_hz_;
    double range_noise_std_;

    // pose
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    bool has_odom_;

    // NEW map params
    double room_half_;
    double inner_span_ratio_;
    std::string inner_open_side_;

    std::vector<Segment> segments_;

    // noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;

    // ros
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JaraSimLidar>());
    rclcpp::shutdown();
    return 0;
}
