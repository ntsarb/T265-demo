#include "sdk_source.h"

#include <algorithm>
#include <cstring>
#include <QThread>
#include <vector>

namespace {

using sdksrc::Group;
using sdksrc::Role;

struct Sel {
    rs2::sensor sensor;
    rs2::stream_profile profile;
    Role role;
};

// Select the profiles we want from any sensor on the device. The T265 can
// expose its streams either as separate sensors (pose/gyro/accel/fisheye*)
// or as one composite "Tracking Module" sensor — this works for both.
std::vector<Sel> pick(const std::vector<rs2::sensor> &sensors)
{
    std::vector<Sel> out;
    auto has = [&](Role r) {
        for (const auto &o : out)
            if (o.role == r)
                return true;
        return false;
    };
    std::vector<Sel> fish;
    for (const auto &s : sensors) {
        auto profiles = s.get_stream_profiles();
        for (const auto &p : profiles) {
            const auto t = p.stream_type();
            if (t == RS2_STREAM_POSE && p.fps() == 200 && !has(Role::Pose)) {
                out.push_back({s, p, Role::Pose});
            } else if (t == RS2_STREAM_FISHEYE) {
                auto vp = p.as<rs2::video_stream_profile>();
                if (vp.width() == 848 && vp.height() == 800 && p.fps() == 30)
                    fish.push_back({s, p, Role::Fish1});
            } else if (t == RS2_STREAM_GYRO && !has(Role::Gyro)) {
                out.push_back({s, p, Role::Gyro});
            } else if (t == RS2_STREAM_ACCEL && !has(Role::Accel)) {
                out.push_back({s, p, Role::Accel});
            }
        }
    }
    std::sort(fish.begin(), fish.end(),
              [](const Sel &a, const Sel &b) {
                  return a.profile.stream_index() < b.profile.stream_index();
              });
    for (size_t i = 0; i < fish.size() && i < 2; ++i)
        out.push_back({fish[i].sensor, fish[i].profile,
                       i == 0 ? Role::Fish1 : Role::Fish2});
    return out;
}

bool profileMatches(const rs2::stream_profile &sel,
                    const rs2::stream_profile &got)
{
    return sel.stream_type() == got.stream_type() &&
           sel.stream_index() == got.stream_index();
}

} // namespace

void SdkSource::sdkLogCallback(rs2_log_severity severity,
                               const rs2_log_message *msg, void *arg)
{
    auto *src = static_cast<SdkSource *>(arg);
    if (!src || !msg || severity < RS2_LOG_SEVERITY_INFO)
        return;
    rs2_error *e = nullptr;
    const char *text = rs2_get_full_log_message(msg, &e);
    if (!text)
        return;
    emit src->statusChanged(QString::fromUtf8(text));
}

void SdkSource::start()
{
    if (running_)
        return;
    running_ = true;
    thread_ = QThread::create([this] { run(); });
    thread_->start();
}

void SdkSource::stop()
{
    running_ = false;
    if (thread_) {
        // Wait first: the QThread must not be deleted while run() can still
        // be inside it. An early-finished thread (device busy) used to be
        // deleted via deleteLater and then wait()'d on here, which crashed.
        if (thread_->wait(5000))
            delete thread_;
        thread_ = nullptr;
    }
}

void SdkSource::run()
{
    using namespace rs2;
    // The SDK log sink is process-global, but the callback below references
    // this object. Detach it on every exit path so it can never fire after
    // this thread (and the object, which it outlives) is gone.
    struct LogScope {
        ~LogScope()
        {
            rs2_error *e = nullptr;
            rs2_log_to_console(RS2_LOG_SEVERITY_ERROR, &e);
            if (e)
                rs2_free_error(e);
        }
    } logScope;
    try {
        rs2_error *e = nullptr;
        // Keep the console quiet; the GUI status bar carries the logs.
        rs2_log_to_console(RS2_LOG_SEVERITY_ERROR, &e);
        error::handle(e);
        rs2_log_to_callback(RS2_LOG_SEVERITY_INFO, &SdkSource::sdkLogCallback,
                            this, &e);
        error::handle(e);

        context ctx;
        emit statusChanged(
            QStringLiteral("Searching for T265 (bootloader firmware is "
                           "uploaded automatically if needed)..."));

        device dev;
        for (int attempt = 0; attempt < 45 && !dev; ++attempt) {
            auto list = ctx.query_devices();
            for (auto d : list) {
                std::string name = d.get_info(RS2_CAMERA_INFO_NAME);
                if (name.find("T265") != std::string::npos) {
                    dev = d;
                    break;
                }
            }
            if (!dev) {
                if (attempt % 5 == 0)
                    emit statusChanged(
                        QStringLiteral("Searching for T265... (%1 s)")
                            .arg(attempt));
                QThread::msleep(1000);
            }
        }

        if (!dev) {
            emit statusChanged(
                QStringLiteral("No T265 found after 45 s. Plug the camera "
                               "into a USB3 port and restart the app."));
            return;
        }

        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        emit statusChanged(
            QStringLiteral("T265 found (serial %1). Starting sensors...")
                .arg(QString::fromStdString(serial)));

        auto sels = pick(dev.query_sensors());
        if (sels.empty()) {
            emit statusChanged(QStringLiteral(
                "No suitable stream profiles on this T265 (wanted pose "
                "200 Hz, fisheye 848x800@30, gyro, accel)."));
            return;
        }

        // Group the selected profiles per sensor.
        std::vector<Group> groups;
        for (const auto &sel : sels) {
            auto it = std::find_if(
                groups.begin(), groups.end(),
                [&](const Group &g) { return g.sensor == sel.sensor; });
            if (it == groups.end())
                groups.push_back({sel.sensor, {sel.profile}, {sel.role}});
            else {
                it->profiles.push_back(sel.profile);
                it->roles.push_back(sel.role);
            }
        }

        std::vector<rs2::sensor> started;
        for (auto &g : groups) {
            try {
                g.sensor.open(g.profiles);
                g.sensor.start([this, &g](rs2::frame f) {
                    handleFrame(f, g);
                });
                started.push_back(g.sensor);
            } catch (const rs2::error &err) {
                emit statusChanged(QStringLiteral("Could not start sensor: %1")
                                       .arg(QString::fromUtf8(err.what())));
            }
        }

        if (started.empty()) {
            emit statusChanged(
                QStringLiteral("No T265 sensors could be started. If the "
                               "device is busy, unplug it, wait 5 s, and "
                               "re-plug (power cycle)."));
            return;
        }

        emit statusChanged(QStringLiteral("Streaming %1 sensor(s) (%2 "
                                          "stream profiles). Move the "
                                          "camera to see its trajectory.")
                              .arg(started.size())
                              .arg(static_cast<int>(sels.size())));

        while (running_)
            QThread::msleep(100);

        for (auto &s : started) {
            try {
                s.stop();
            } catch (...) {
            }
            try {
                s.close();
            } catch (...) {
            }
        }
    } catch (const rs2::error &err) {
        emit statusChanged(
            QStringLiteral("SDK error: %1")
                .arg(QString::fromUtf8(err.what())));
    } catch (const std::exception &err) {
        emit statusChanged(
            QStringLiteral("Error: %1")
                .arg(QString::fromUtf8(err.what())));
    }
}

void SdkSource::handleFrame(rs2::frame f, const Group &g)
{
    const auto got = f.get_profile();
    Role role = Role::Pose;
    for (size_t i = 0; i < g.profiles.size(); ++i) {
        if (profileMatches(g.profiles[i], got)) {
            role = g.roles[i];
            break;
        }
    }
    try {
        switch (role) {
        case Role::Fish1:
        case Role::Fish2: {
            auto vf = f.as<rs2::video_frame>();
            const int w = vf.get_width();
            const int h = vf.get_height();
            const int stride = vf.get_stride_in_bytes();
            auto data = static_cast<const uint8_t *>(f.get_data());
            QImage img(w, h, QImage::Format_Grayscale8);
            for (int row = 0; row < h; ++row)
                std::memcpy(img.scanLine(row),
                            data + static_cast<size_t>(row) * stride, w);
            if (role == Role::Fish1)
                emit image1Ready(img);
            else
                emit image2Ready(img);
            break;
        }
        case Role::Pose: {
            auto pf = f.as<rs2::pose_frame>();
            const auto p = pf.get_pose_data();
            PoseSample s;
            s.t = f.get_timestamp() / 1000.0;
            s.x = p.translation.x;
            s.y = p.translation.y;
            s.z = p.translation.z;
            s.qx = p.rotation.x;
            s.qy = p.rotation.y;
            s.qz = p.rotation.z;
            s.qw = p.rotation.w;
            s.confidence = static_cast<int>(p.tracker_confidence);
            emit poseReady(s);
            break;
        }
        case Role::Gyro: {
            auto mf = f.as<rs2::motion_frame>();
            const auto v = mf.get_motion_data();
            ImuSample s;
            s.t = f.get_timestamp() / 1000.0;
            s.gx = v.x;
            s.gy = v.y;
            s.gz = v.z;
            emit gyroReady(s);
            break;
        }
        case Role::Accel: {
            auto mf = f.as<rs2::motion_frame>();
            const auto v = mf.get_motion_data();
            ImuSample s;
            s.t = f.get_timestamp() / 1000.0;
            s.ax = v.x;
            s.ay = v.y;
            s.az = v.z;
            emit accelReady(s);
            break;
        }
        }
    } catch (const std::exception &) {
        // drop malformed frames
    }
}
