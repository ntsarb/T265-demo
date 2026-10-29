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
    auto *t = QThread::create([this] { run(); });
    connect(t, &QThread::finished, t, &QObject::deleteLater);
    thread_ = t;
    t->start();
}

void RosSource::stop()
{
    running_ = false;
    if (thread_) {
        thread_->wait(5000);
        thread_ = nullptr;
    }
}

void RosSource::run()
{
    try {
        static std::once_flag init_flag;
        std::call_once(init_flag, [] { rclcpp::init(0, nullptr); });

        auto node = std::make_shared<rclcpp::Node>("t265_demo");
        const std::string p = prefix_.toStdString();
        // The wrapper publishes all streams best-effort, so subscribe with
        // SensorDataQoS (a best-effort sub matches both pub types).
        auto qos = rclcpp::SensorDataQoS();

        // Store the shared_ptrs: dropping them destroys the subscription.
        auto sub_pose = node->create_subscription<nav_msgs::msg::Odometry>(
            p + "/pose/sample", qos,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr &m) {
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
                if (!img.isNull())
                    emit image1Ready(img);
            });

        const std::string t2 = p + "/fisheye2/image_rect_raw";
        auto sub_img2 = node->create_subscription<sensor_msgs::msg::Image>(
            t2, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &m) {
                auto img = toImage(m);
                if (!img.isNull())
                    emit image2Ready(img);
            });

        auto sub_gyro = node->create_subscription<sensor_msgs::msg::Imu>(
            p + "/gyro/sample", qos,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr &m) {
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
                ImuSample s;
                s.t = stampSeconds(m->header.stamp);
                s.ax = m->linear_acceleration.x;
                s.ay = m->linear_acceleration.y;
                s.az = m->linear_acceleration.z;
                emit accelReady(s);
            });

        emit statusChanged(
            QStringLiteral("ROS mode: connected, waiting for %1/pose/sample ... "
                           "(launch the wrapper node first if it is not "
                           "running)")
                .arg(prefix_));

        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);
        while (running_) {
            exec.spin_some(std::chrono::milliseconds(5));
        }
        rclcpp::shutdown();
    } catch (const std::exception &err) {
        emit statusChanged(
            QStringLiteral("ROS error: %1")
                .arg(QString::fromUtf8(err.what())));
    }
}
