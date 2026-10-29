#include "gui_window.h"

#include <cmath>

#include <QApplication>
#include <QFileDialog>
#include <QPaintEvent>
#include <QPainter>
#include <QStatusBar>
#include <QHBoxLayout>
#include <QPalette>
#include <QTextStream>
#include <QVBoxLayout>
#include <QtMath>

// Live image label: keeps the full-resolution frame and scales it in
// paintEvent to fit the current widget size. Unlike QLabel::setPixmap,
// showing an image never changes the label's size hints, so the window
// geometry stays stable and resizing scales the picture both ways.
// (Forward-declared in gui_window.h; must be defined at global scope.)
class ImageLabel : public QLabel
{
public:
    explicit ImageLabel(const QString &placeholder, QWidget *parent = nullptr)
        : QLabel(placeholder, parent), placeholder_(placeholder)
    {
        setMinimumSize(432, 410);
        setAlignment(Qt::AlignCenter);
        setFrameShape(QFrame::StyledPanel);
    }

    void setImage(const QImage &img)
    {
        img_ = img;
        pmCacheSize_ = QSize(); // invalidate the scaled-pixmap cache
        update();
    }

    // Size hints are fixed: the stream must never resize the window.
    QSize minimumSizeHint() const override { return minimumSize(); }
    QSize sizeHint() const override { return minimumSize(); }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        const QRect r = rect().adjusted(1, 1, -1, -1);
        if (img_.isNull()) {
            p.fillRect(r, palette().window());
            p.setPen(QColor(150, 160, 180));
            p.drawText(r, Qt::AlignCenter, placeholder_);
            return;
        }
        p.fillRect(r, Qt::black);
        if (pm_.isNull() || pmCacheSize_ != r.size()) {
            pm_ = QPixmap::fromImage(img_).scaled(
                r.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            pmCacheSize_ = r.size();
        }
        p.drawPixmap(r.x() + (r.width() - pm_.width()) / 2,
                     r.y() + (r.height() - pm_.height()) / 2, pm_);
    }

private:
    QString placeholder_;
    QImage img_;
    QPixmap pm_;
    QSize pmCacheSize_;
};

GuiWindow::GuiWindow(ISensorSource *source, QWidget *parent)
    : QMainWindow(parent), src_(source)
{
    QPalette pal = qApp->palette();
    pal.setColor(QPalette::Window, QColor(30, 32, 40));
    pal.setColor(QPalette::Base, QColor(24, 26, 34));
    pal.setColor(QPalette::Text, QColor(225, 230, 240));
    qApp->setPalette(pal);
    setStyleSheet(
        "QMainWindow, QWidget { background: #1e2028; color: #e1e6f0; }"
        "QLabel { color: #c8cdd8; }"
        "QPushButton { background: #34405a; border: 1px solid #4a5878;"
        " padding: 6px 14px; border-radius: 4px; }"
        "QPushButton:hover { background: #42527a; }");

    img1_ = new ImageLabel(QStringLiteral("fisheye 1 (no data)"));
    img2_ = new ImageLabel(QStringLiteral("fisheye 2 (no data)"));

    traj_ = new TrajectoryCanvas;
    gyro_ = new ImuStrip(ImuStrip::Gyro);
    accel_ = new ImuStrip(ImuStrip::Accel);
    readout_ = new QLabel(QStringLiteral("position: —"));
    readout_->setStyleSheet("color: #9fd6ff;");
    // Fixed widths: the text length varies, and a growing label would grow
    // the window. Oversized text is clipped; the window stays put.
    readout_->setFixedWidth(620);
    hz_ = new QLabel(QStringLiteral("pose -- Hz   gyro -- Hz   accel -- Hz"));
    hz_->setFixedWidth(430);

    auto *clearBtn = new QPushButton(QStringLiteral("Clear trajectory"));
    auto *csvBtn = new QPushButton(QStringLiteral("Save CSV..."));
    connect(clearBtn, &QPushButton::clicked, this, [this] {
        traj_->clear();
        gyro_->clear();
        accel_->clear();
    });
    connect(csvBtn, &QPushButton::clicked, this, [this] {
        const QString file = QFileDialog::getSaveFileName(
            this, QStringLiteral("Save pose CSV"), "t265-pose.csv",
            QStringLiteral("CSV files (*.csv)"));
        if (file.isEmpty())
            return;
        const QString csv = exportCsv();
        if (csv.isEmpty()) {
            statusBar()->showMessage(QStringLiteral("nothing to export yet"));
            return;
        }
        FILE *f = fopen(file.toUtf8().constData(), "w");
        if (!f) {
            statusBar()->showMessage(QStringLiteral("cannot open file"));
            return;
        }
        fputs(csv.toUtf8().constData(), f);
        fclose(f);
        statusBar()->showMessage(QStringLiteral("saved %1").arg(file), 5000);
    });

    auto *left = new QVBoxLayout;
    left->setContentsMargins(0, 0, 0, 0);
    left->setSpacing(6);
    left->addWidget(new QLabel(QStringLiteral("fisheye 1")));
    left->addWidget(img1_);
    left->addWidget(new QLabel(QStringLiteral("fisheye 2")));
    left->addWidget(img2_);

    auto *row1 = new QHBoxLayout;
    row1->addLayout(left);
    row1->addWidget(traj_, 1);

    auto *row2 = new QHBoxLayout;
    row2->addWidget(gyro_);
    row2->addWidget(accel_);
    row2->addStretch(1);
    row2->addWidget(clearBtn);
    row2->addWidget(csvBtn);

    auto *titleRow = new QHBoxLayout;
    titleRow->addWidget(new QLabel(QStringLiteral("T265 demo")));
    titleRow->addStretch(1);
    titleRow->addWidget(readout_);
    titleRow->addWidget(hz_);

    auto *root = new QVBoxLayout;
    root->addLayout(titleRow);
    root->addLayout(row1, 1);
    root->addLayout(row2);

    auto *central = new QWidget;
    central->setLayout(root);
    setCentralWidget(central);
    setWindowTitle(QStringLiteral("T265 demo — %1").arg(src_->describe()));

    connect(src_, &ISensorSource::poseReady, this, &GuiWindow::onPose);
    connect(src_, &ISensorSource::gyroReady, this, &GuiWindow::onGyro);
    connect(src_, &ISensorSource::accelReady, this, &GuiWindow::onAccel);
    connect(src_, &ISensorSource::image1Ready, this, &GuiWindow::onImage1);
    connect(src_, &ISensorSource::image2Ready, this, &GuiWindow::onImage2);
    connect(src_, &ISensorSource::statusChanged, this, &GuiWindow::onStatus);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &GuiWindow::tick);
    timer_->start(33);
}

void GuiWindow::onPose(const PoseSample &s)
{
    // The T265 firmware emits pose frames with NaN fields while the
    // tracker is lost/re-locking; drop them so the trajectory, readout
    // and CSV export stay finite.
    if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.z) ||
        !std::isfinite(s.qx) || !std::isfinite(s.qy) || !std::isfinite(s.qz) ||
        !std::isfinite(s.qw))
        return;
    poseCnt_++;
    totalPose_++;
    traj_->append(s);
    const double yaw = std::atan2(2.0 * (s.qw * s.qz + s.qx * s.qy),
                                  1.0 - 2.0 * (s.qy * s.qy + s.qz * s.qz));
    const double pitch =
        std::asin(2.0 * (s.qw * s.qy - s.qz * s.qx));
    const double roll = std::atan2(2.0 * (s.qw * s.qx + s.qy * s.qz),
                                   1.0 - 2.0 * (s.qx * s.qx + s.qy * s.qy));
    readout_->setText(
        QStringLiteral("pos (%1, %2, %3) m   yaw/pitch/roll %4/%5/%6 deg   "
                       "conf %7")
            .arg(s.x, 0, 'f', 3).arg(s.y, 0, 'f', 3).arg(s.z, 0, 'f', 3)
             .arg(qRadiansToDegrees(yaw), 0, 'f', 1)
             .arg(qRadiansToDegrees(pitch), 0, 'f', 1)
             .arg(qRadiansToDegrees(roll), 0, 'f', 1)
            .arg(s.confidence));
}

void GuiWindow::onGyro(const ImuSample &s)
{
    gyroCnt_++;
    totalGyro_++;
    gyro_->append(s);
}

void GuiWindow::onAccel(const ImuSample &s)
{
    accelCnt_++;
    totalAccel_++;
    accel_->append(s);
}

void GuiWindow::onImage1(const QImage &img)
{
    img1Cnt_++;
    totalImg1_++;
    img1_->setImage(img);
}

void GuiWindow::onImage2(const QImage &img)
{
    img2Cnt_++;
    totalImg2_++;
    img2_->setImage(img);
}

void GuiWindow::onStatus(const QString &msg)
{
    statusBar()->showMessage(msg);
}

void GuiWindow::tick()
{
    static int tick = 0;
    tick++;
    if (tick % 15 == 0) { // ~5 Hz label refresh
        hz_->setText(QStringLiteral("pose %1 Hz   gyro %2 Hz   accel %3 Hz   "
                                    "cam1 %4 Hz   cam2 %5 Hz")
                         .arg(poseCnt_ * 2)
                         .arg(gyroCnt_ * 2)
                         .arg(accelCnt_ * 2)
                         .arg(img1Cnt_ * 2)
                         .arg(img2Cnt_ * 2));
        poseCnt_ = gyroCnt_ = accelCnt_ = img1Cnt_ = img2Cnt_ = 0;
    }
    traj_->update();
}

QString GuiWindow::frameCounts() const
{
    return QStringLiteral("pose=%1 cam1=%2 cam2=%3 gyro=%4 accel=%5")
        .arg(totalPose_)
        .arg(totalImg1_)
        .arg(totalImg2_)
        .arg(totalGyro_)
        .arg(totalAccel_);
}

int GuiWindow::dumpCsv(const QString &path)
{
    const QString csv = exportCsv();
    if (csv.isEmpty())
        return 0;
    FILE *f = fopen(path.toUtf8().constData(), "w");
    if (!f)
        return 0;
    fputs(csv.toUtf8().constData(), f);
    fclose(f);
    return traj_->samples().size() - 1;
}

QString GuiWindow::exportCsv()
{
    const auto &samples = traj_->samples();
    if (samples.empty())
        return {};
    QString out = "t,x,y,z,qx,qy,qz,qw\n";
    QTextStream ts(&out);
    ts.setRealNumberPrecision(9);
    for (const auto &s : samples)
        ts << s.t << ',' << s.x << ',' << s.y << ',' << s.z << ',' << s.qx
           << ',' << s.qy << ',' << s.qz << ',' << s.qw << '\n';
    return out;
}
