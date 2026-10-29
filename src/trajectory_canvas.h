#pragma once

#include <deque>
#include <QWidget>
#include "source.h"

class TrajectoryCanvas : public QWidget
{
    Q_OBJECT
public:
    explicit TrajectoryCanvas(QWidget *parent = nullptr);

    void clear();
    void append(const PoseSample &p);
    const std::deque<PoseSample> &samples() const { return buf_; }

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void wheelEvent(QWheelEvent *) override;

private:
    std::deque<PoseSample> buf_;
    // Position of the camera at the last "Clear": all samples are stored
    // relative to it, so a cleared run starts at (0,0,0).
    double offsetX_ = 0.0, offsetY_ = 0.0, offsetZ_ = 0.0;
    double yawDeg_ = 30.0;
    double pitchDeg_ = -30.0;
    double zoom_ = 1.0;
    int lastX_ = -1;

    static constexpr size_t kMaxSamples = 12000; // 60 s @ 200 Hz
};
