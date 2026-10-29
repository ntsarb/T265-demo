#pragma once

#include <QObject>
#include <QImage>
#include <QString>

struct PoseSample {
    double t = 0.0;
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    int confidence = -1; // -1 unknown, 0 failed .. 3 high (SDK)
};

struct ImuSample {
    double t = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double ax = 0.0, ay = 0.0, az = 0.0;
};

// Common interface for the two data sources (direct SDK / ROS topics).
// Signals are emitted from the source's worker thread; the GUI receives
// them on the main thread via queued connections.
class ISensorSource : public QObject
{
    Q_OBJECT
public:
    explicit ISensorSource(QObject *parent = nullptr) : QObject(parent) {}
    ~ISensorSource() override = default;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual QString describe() const = 0;

signals:
    void image1Ready(const QImage &img);
    void image2Ready(const QImage &img);
    void poseReady(const PoseSample &p);
    void gyroReady(const ImuSample &g);
    void accelReady(const ImuSample &a);
    void statusChanged(const QString &status);
};
