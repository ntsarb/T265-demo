#pragma once

#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include "imu_strip.h"
#include "source.h"
#include "trajectory_canvas.h"

class ImageLabel;

class GuiWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit GuiWindow(ISensorSource *source, QWidget *parent = nullptr);

private slots:
    void onPose(const PoseSample &s);
    void onGyro(const ImuSample &s);
    void onAccel(const ImuSample &s);
    void onImage1(const QImage &img);
    void onImage2(const QImage &img);
    void onStatus(const QString &msg);
    void tick();

public:
    int dumpCsv(const QString &path); // returns row count
    QString frameCounts() const;

private:
    QString exportCsv();

    ISensorSource *src_;
    ImageLabel *img1_;
    ImageLabel *img2_;
    TrajectoryCanvas *traj_;
    ImuStrip *gyro_;
    ImuStrip *accel_;
    QLabel *readout_;
    QLabel *hz_;
    QTimer *timer_;

    int poseCnt_ = 0, gyroCnt_ = 0, accelCnt_ = 0, img1Cnt_ = 0, img2Cnt_ = 0;
    qint64 totalPose_ = 0, totalGyro_ = 0, totalAccel_ = 0;
    qint64 totalImg1_ = 0, totalImg2_ = 0;
};
