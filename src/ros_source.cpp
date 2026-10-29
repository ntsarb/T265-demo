#include "ros_source.h"

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

QImage toImage(const sensor_msgs::msg::Image::ConstSharedPtr &m)
{
    const int w = static_cast<int>(m->width);
    const int h = static_cast<int>(m->height);
    QImage img;
    if (m->encoding == "mono8") {
        img = QImage(w, h, QImage::Format_Grayscale8);
        const uint8_t *d = m->data.data();
        for (int row = 0; row < h; ++row)
            std::memcpy(img.scanLine(row), d + static_cast<size_t>(row) * m->step,
                        w);
    } else if (m->encoding == "rgb8") {
        img = QImage(w, h, QImage::Format_RGB888);
        const uint8_t *d = m->data.data();
        for (int row = 0; row < h; ++row)
            std::memcpy(img.scanLine(row), d + static_cast<size_t>(row) * m->step,
                        static_cast<size_t>(w) * 3);
    }
    return img;
}

double stampSeconds(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

} // namespace

void RosSource::start()
{
    if (running_)
        return;
    running_ = true;
    thread_ = QThread::create([this] { run(); });
    thread_->start();
}

void RosSource::stop()
{
    running_ = false;
    if (thread_) {
        // Wait first: the QThread must not be deleted while run() can still
        // be inside it (same early-finish crash that hit the SDK source).
        if (thread_->wait(5000))
            delete thread_;
        thread_ = nullptr;
    }
}

void RosSource::run()
{
    try {
        static std::once_flag init_flag;
        std::call_once(init_flag, [] { rclcpp::init(0, nullptr); });

        // Fast DDS' shared-memory transport needs inotify watches per
        // channel; under a tight watch budget (see fs.inotify.max_user_watches)
        // participant creation can transiently fail, so retry before giving up.
        std::shared_ptr<rclcpp::Node> node;
        for (int attempt = 0; ; ++attempt) {
            try {
                node = std::make_shared<rclcpp::Node>("t265_demo");
                break;
            } catch (const std::exception &) {
                if (attempt >= 5)
                    throw;
                QThread::msleep(1000);
            }
        }
        const std::string p = prefix_.toStdString();
        // The wrapper publishes all streams best-effort, so subscribe with
        // SensorDataQoS (a best-effort sub matches both pub types).
        auto qos = rclcpp::SensorDataQoS();

        // Store the shared_ptrs: dropping them destroys the subscription.
        auto sub_pose = node->create_subscription<nav_msgs::msg::Odometry>(
            p + "/pose/sample", qos,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr &m) {
                frames_++;
                PoseSample s;
                s.t = stampSeconds(m->header.stamp);
                s.x = m->pose.pose.position.x;
                s.y = m->pose.pose.position.y;
                s.z = m->pose.pose.position.z;
                s.qx = m->pose.pose.orientation.x;
                s.qy = m->pose.pose.orientation.y;
                s.qz = m->pose.pose.orientation.z;
                s.qw = m->pose.pose.orientation.w;
                emit poseReady(s);
            });

        const std::string t1 = p + "/fisheye1/image_rect_raw";
        auto sub_img1 = node->create_subscription<sensor_msgs::msg::Image>(
            t1, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &m) {
                auto img = toImage(m);
                if (!img.isNull()) {
                    frames_++;
                    emit image1Ready(img);
                }
            });

        const std::string t2 = p + "/fisheye2/image_rect_raw";
        auto sub_img2 = node->create_subscription<sensor_msgs::msg::Image>(
            t2, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &m) {
                auto img = toImage(m);
                if (!img.isNull()) {
                    frames_++;
                    emit image2Ready(img);
                }
            });

        auto sub_gyro = node->create_subscription<sensor_msgs::msg::Imu>(
            p + "/gyro/sample", qos,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr &m) {
                frames_++;
                ImuSample s;
                s.t = stampSeconds(m->header.stamp);
                s.gx = m->angular_velocity.x;
                s.gy = m->angular_velocity.y;
                s.gz = m->angular_velocity.z;
                emit gyroReady(s);
            });

        auto sub_accel = node->create_subscription<sensor_msgs::msg::Imu>(
            p + "/accel/sample", qos,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr &m) {
                frames_++;
                ImuSample s;
                s.t = stampSeconds(m->header.stamp);
                s.ax = m->linear_acceleration.x;
                s.ay = m->linear_acceleration.y;
                s.az = m->linear_acceleration.z;
                emit accelReady(s);
            });

        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);

        // Poll so the footer can say whether the wrapper node is actually
        // running, instead of just waiting silently. Receiving frames is the
        // ground truth (graph introspection is unreliable across some DDS
        // configurations), the graph is only consulted while nothing flows.
        const std::string pose_topic = p + "/pose/sample";
        auto last_check = std::chrono::steady_clock::now();
        int last_state = -1; // -1 unknown, 0 absent, 1 node only, 2 pub, 3 data
        auto poll = [&]() {
            try {
                const int state =
                    frames_ > 0 ? 3 :
                    node->count_publishers(pose_topic) > 0 ? 2 :
                    [](const auto &node, const std::string &p) {
                        try {
                            for (const auto &full : node->get_node_names())
                                if (full == p)
                                    return 1;
                        } catch (const std::exception &) {
                        }
                        return 0;
                    }(node, p) ? 1 : 0;
                if (state != last_state) {
                    last_state = state;
                    if (state == 3)
                        emit statusChanged(
                            QStringLiteral("ROS mode: wrapper running — "
                                           "receiving data"));
                    else if (state == 2)
                        emit statusChanged(
                            QStringLiteral("ROS mode: wrapper is publishing "
                                           "%1, waiting for the first frame...")
                                .arg(QString::fromStdString(pose_topic)));
                    else if (state == 1)
                        emit statusChanged(
                            QStringLiteral("ROS mode: wrapper node found, but "
                                           "%1 has no publisher yet")
                                .arg(QString::fromStdString(pose_topic)));
                    else
                        emit statusChanged(
                            QStringLiteral("ROS mode: wrapper not running — "
                                           "launch it first: ros2 launch "
                                           "realsense2_camera rs_launch.py"));
                }
            } catch (const std::exception &) {
            }
        };

        emit statusChanged(
            QStringLiteral("ROS mode: connected, checking the ROS graph for "
                           "the wrapper node..."));
        poll();
        while (running_) {
            exec.spin_some(std::chrono::milliseconds(5));
            const auto now = std::chrono::steady_clock::now();
            if (now - last_check >= std::chrono::seconds(1)) {
                last_check = now;
                poll();
            }
        }
        rclcpp::shutdown();
    } catch (const std::exception &err) {
        emit statusChanged(
            QStringLiteral("ROS error: %1")
                .arg(QString::fromUtf8(err.what())));
    }
}
