#include "trajectory_canvas.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <QMouseEvent>
#include <QPainter>
#include <QtMath>
#include <QWheelEvent>

TrajectoryCanvas::TrajectoryCanvas(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(420, 320);
    setMouseTracking(true);
    setCursor(Qt::OpenHandCursor);
}

void TrajectoryCanvas::clear()
{
    if (!buf_.empty()) {
        // Re-base: the camera's position at this instant becomes the new
        // (0,0,0), so the next run starts at the origin.
        const auto &last = buf_.back();
        offsetX_ = last.x;
        offsetY_ = last.y;
        offsetZ_ = last.z;
    } else {
        offsetX_ = offsetY_ = offsetZ_ = 0.0;
    }
    buf_.clear();
    // Return the virtual camera to its home state: origin (0,0,0) at the
    // center, default orientation and zoom.
    yawDeg_ = 30.0;
    pitchDeg_ = -30.0;
    zoom_ = 1.0;
    update();
}

void TrajectoryCanvas::append(const PoseSample &p)
{
    buf_.push_back(PoseSample{p.t, p.x - offsetX_, p.y - offsetY_,
                              p.z - offsetZ_, p.qx, p.qy, p.qz, p.qw,
                              p.confidence});
    while (buf_.size() > kMaxSamples)
        buf_.pop_front();
}

void TrajectoryCanvas::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
        lastX_ = e->position().toPoint().x();
        setCursor(Qt::ClosedHandCursor);
    }
}

void TrajectoryCanvas::mouseReleaseEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
        lastX_ = -1;
        setCursor(Qt::OpenHandCursor);
    }
}

void TrajectoryCanvas::mouseMoveEvent(QMouseEvent *e)
{
    if (lastX_ >= 0 && (e->buttons() & Qt::LeftButton)) {
        const int dx = e->position().toPoint().x() - lastX_;
        yawDeg_ += dx * 0.4;
        lastX_ = e->position().toPoint().x();
        update();
    }
}

void TrajectoryCanvas::wheelEvent(QWheelEvent *e)
{
    const int dy = e->angleDelta().y();
    if (dy > 0)
        zoom_ *= 1.1;
    else if (dy < 0)
        zoom_ /= 1.1;
    zoom_ = std::clamp(zoom_, 0.2, 50.0);
    update();
}

void TrajectoryCanvas::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(24, 26, 34));

    p.setPen(QColor(150, 160, 180));
    p.drawText(rect().adjusted(10, 6, -10, -6), Qt::AlignLeft | Qt::AlignTop,
               QStringLiteral("T265 pose trajectory  (drag: rotate  |  wheel: zoom)"));

    double minX = 0, minY = 0, minZ = 0;
    double maxX = 0, maxY = 0, maxZ = 0;
    for (const auto &s : buf_) {
        minX = std::min(minX, s.x);
        maxX = std::max(maxX, s.x);
        minY = std::min(minY, s.y);
        maxY = std::max(maxY, s.y);
        minZ = std::min(minZ, s.z);
        maxZ = std::max(maxZ, s.z);
    }
    const double range =
        std::max({maxX - minX, maxY - minY, maxZ - minZ, 0.3});
    const double cx = (minX + maxX) / 2.0;
    const double cy = (minY + maxY) / 2.0;
    const double cz = (minZ + maxZ) / 2.0;
    const double scale =
        (std::min(width(), height()) * 0.40) / range * zoom_;

    // Defensive: a non-finite scale would make every projected point NaN
    // (Qt would warn about degenerate arcs and draw nothing).
    if (!std::isfinite(scale)) {
        p.drawText(rect().adjusted(0, 0, 0, -30), Qt::AlignCenter,
                   QStringLiteral("trajectory data invalid"));
        return;
    }

    const double cyaw = std::cos(qDegreesToRadians(yawDeg_));
    const double syaw = std::sin(qDegreesToRadians(yawDeg_));
    const double cpit = std::cos(qDegreesToRadians(pitchDeg_));
    const double spit = std::sin(qDegreesToRadians(pitchDeg_));

    auto toView = [&](double x, double y, double z) {
        const double x1 = (x - cx) * cyaw - (y - cy) * syaw;
        const double y1 = (x - cx) * syaw + (y - cy) * cyaw;
        const double z1 = z - cz;
        const double y2 = y1 * cpit - z1 * spit;
        const double z2 = y1 * spit + z1 * cpit;
        return std::array<double, 3>{x1, y2, z2};
    };
    auto toScreen = [&](const std::array<double, 3> &v) {
        return QPointF(width() / 2.0 + v[0] * scale,
                       height() / 2.0 - v[2] * scale);
    };

    auto drawAxis = [&](double ax, double ay, double az, const QColor &col,
                        const char *label) {
        const double L = range * 0.6;
        auto o = toScreen(toView(0, 0, 0));
        auto e = toScreen(toView(ax * L, ay * L, az * L));
        p.setPen(QPen(col, 1));
        p.drawLine(o, e);
        p.setPen(col);
        p.drawText(e + QPointF(4, 4), label);
    };
    drawAxis(1, 0, 0, QColor(220, 90, 90), "x");
    drawAxis(0, 1, 0, QColor(90, 200, 110), "y");
    drawAxis(0, 0, 1, QColor(90, 140, 230), "z");

    const size_t n = buf_.size();
    if (n >= 2) {
        const size_t step = n > 4000 ? 3 : 1;
        for (size_t i = step; i < n; i += step) {
            const auto &a = buf_[i - step];
            const auto &b = buf_[i];
            const auto pa = toScreen(toView(a.x, a.y, a.z));
            const auto pb = toScreen(toView(b.x, b.y, b.z));
            const double age = 1.0 - double(i) / double(n);
            QColor col(120 + int(135 * age), 160 + int(60 * age), 255);
            col.setAlphaF(0.25 + 0.75 * age);
            p.setPen(QPen(col, 1.6));
            p.drawLine(pa, pb);
        }

        const auto first = toScreen(
            toView(buf_.front().x, buf_.front().y, buf_.front().z));
        const auto last =
            toScreen(toView(buf_.back().x, buf_.back().y, buf_.back().z));
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(90, 220, 120));
        p.drawEllipse(first, 4, 4);
        p.setBrush(QColor(255, 90, 90));
        p.drawEllipse(last, 5, 5);
    } else {
        // Empty state: show the origin marker so the virtual camera is
        // visibly home at (0,0,0).
        const auto o = toScreen(toView(0, 0, 0));
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(225, 230, 240));
        p.drawEllipse(o, 4, 4);
        p.setPen(QColor(110, 120, 140));
        p.drawText(rect().adjusted(0, 0, 0, -30), Qt::AlignCenter,
                   QStringLiteral("no trajectory  (camera at origin)"));
    }
}
