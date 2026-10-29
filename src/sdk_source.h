#pragma once

#include <atomic>
#include <QThread>
#include <librealsense2/rs.hpp>
#include <vector>
#include "source.h"

namespace sdksrc {
enum class Role { Pose, Fish1, Fish2, Gyro, Accel };
struct Group {
    rs2::sensor sensor;
    std::vector<rs2::stream_profile> profiles;
    std::vector<Role> roles; // same order as profiles
};
} // namespace sdksrc

class SdkSource : public ISensorSource
{
    Q_OBJECT
public:
    explicit SdkSource(QObject *parent = nullptr) : ISensorSource(parent) {}
    ~SdkSource() override { stop(); }

    void start() override;
    void stop() override;
    QString describe() const override
    {
        return QStringLiteral("direct librealsense SDK (T265-capable build)");
    }

    static void sdkLogCallback(rs2_log_severity severity,
                               const rs2_log_message *msg, void *arg);

private:
    void run();
    void handleFrame(rs2::frame f, const sdksrc::Group &g);

    std::atomic<bool> running_{false};
    QThread *thread_ = nullptr;
};
