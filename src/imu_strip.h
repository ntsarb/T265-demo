#pragma once

#include <deque>
#include <QWidget>
#include "source.h"

class ImuStrip : public QWidget
{
    Q_OBJECT
public:
    enum Kind { Gyro, Accel };

    explicit ImuStrip(Kind kind, QWidget *parent = nullptr);

    void append(const ImuSample &s);
    void clear();

protected:
    void paintEvent(QPaintEvent *) override;

private:
    Kind kind_;
    std::deque<double> xs_, ys_, zs_;

    static constexpr size_t kMax = 400;
};
