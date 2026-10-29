#include "imu_strip.h"

#include <algorithm>
#include <cmath>
#include <QPainter>

ImuStrip::ImuStrip(Kind kind, QWidget *parent)
    : QWidget(parent), kind_(kind)
{
    setMinimumSize(360, 90);
}

void ImuStrip::append(const ImuSample &s)
{
    if (kind_ == Gyro) {
        xs_.push_back(s.gx);
        ys_.push_back(s.gy);
        zs_.push_back(s.gz);
    } else {
        xs_.push_back(s.ax);
        ys_.push_back(s.ay);
        zs_.push_back(s.az);
    }
    while (xs_.size() > kMax) {
        xs_.pop_front();
        ys_.pop_front();
        zs_.pop_front();
    }
    update();
}

void ImuStrip::clear()
{
    xs_.clear();
    ys_.clear();
    zs_.clear();
    update();
}

void ImuStrip::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(24, 26, 34));

    const bool gyro = kind_ == Gyro;
    const int fh = p.fontMetrics().height();
    p.setPen(QColor(150, 160, 180));
    p.drawText(10, fh + 4,
               (gyro ? QStringLiteral("gyro  (rad/s)   ")
                     : QStringLiteral("accel  (m/s^2)   ")) +
                   QStringLiteral("x/y/z = red/green/blue"));

    if (xs_.size() < 2) {
        p.drawText(rect().adjusted(0, fh + 12, 0, -fh - 12), Qt::AlignCenter,
                   QStringLiteral("waiting for data..."));
        return;
    }

    double lo = 1e9, hi = -1e9;
    for (const auto &v : {&xs_, &ys_, &zs_})
        for (double d : *v) {
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
    // Floor the span so a still camera does not auto-zoom into pure sensor
    // noise: gyro noise ~0.02 rad/s, accel gravity ~9.8 m/s^2 per axis.
    const double minSpan = gyro ? 0.4 : 4.0;
    const double mid = 0.5 * (lo + hi);
    const double span = std::max(hi - lo, minSpan);
    lo = mid - span / 2;
    hi = mid + span / 2;
    const int w = width();
    const int h = height();
    const int top = fh + 10;
    const int bottom = h - fh - 8;
    auto Y = [&](double v) {
        return bottom - (v - lo) / (hi - lo) * (bottom - top);
    };
    const size_t n = xs_.size();
    auto X = [&](size_t i, size_t n) {
        return static_cast<int>(i * (w - 12) / std::max<size_t>(n - 1, 1)) +
               6;
    };

    p.setPen(QPen(QColor(90, 95, 110), 1, Qt::DashLine));
    const int zy = static_cast<int>(Y(0.0));
    if (zy > top && zy < bottom)
        p.drawLine(6, zy, w - 6, zy);

    auto draw = [&](const std::deque<double> &v, const QColor &col) {
        QPen pen(col, 1.2);
        p.setPen(pen);
        for (size_t i = 1; i < n; ++i)
            p.drawLine(QPoint(X(i - 1, n), Y(v[i - 1])),
                       QPoint(X(i, n), Y(v[i])));
    };
    draw(xs_, QColor(235, 90, 90));
    draw(ys_, QColor(90, 210, 120));
    draw(zs_, QColor(90, 140, 230));

    p.setPen(QColor(200, 205, 220));
    p.drawText(8, h - 4,
               QStringLiteral("lo %1  hi %2")
                   .arg(lo, 0, 'f', 3)
                   .arg(hi, 0, 'f', 3));
}
