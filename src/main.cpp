#include <QApplication>
#include <QTimer>

#include <cstdio>

#include "gui_window.h"
#include "sdk_source.h"
#ifdef DEMO_HAS_ROS
#include "ros_source.h"
#endif

static int printUsage()
{
    fprintf(stderr,
            "t265-demo — Intel RealSense T265 viewer\n"
            "usage: t265-demo [--mode sdk|ros] [--topic-prefix PREFIX]\n"
            "                [--dump-shot FILE] [--dump-delay SECS]\n"
            "  --mode sdk        talk to the camera via librealsense (default)\n"
            "  --mode ros        read ROS 2 topics (requires a running wrapper "
            "node)\n"
            "  --topic-prefix P  topic prefix, default /camera/camera\n"
            "  --dump-shot FILE  save the window to FILE as PNG after a delay "
            "and quit\n"
            "  --dump-csv FILE   also save the pose buffer to FILE (CSV)\n"
            "  --dump-delay SEC  delay for --dump-shot, default 25 s\n");
    return 1;
}

int main(int argc, char **argv)
{
    QString mode = "sdk";
    QString prefix = "/camera/camera";
    QString dumpShot, dumpCsv;
    int dumpDelaySec = 25;
    for (int i = 1; i < argc; ++i) {
        const QString a = argv[i];
        if (a == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (a == "--topic-prefix" && i + 1 < argc) {
            prefix = argv[++i];
        } else if (a == "--dump-shot" && i + 1 < argc) {
            dumpShot = argv[++i];
        } else if (a == "--dump-csv" && i + 1 < argc) {
            dumpCsv = argv[++i];
        } else if (a == "--dump-delay" && i + 1 < argc) {
            dumpDelaySec = atoi(argv[++i]);
        } else if (a == "--help" || a == "-h") {
            return printUsage();
        } else {
            return printUsage();
        }
    }

    QApplication app(argc, argv);
    app.setApplicationName("t265-demo");

    std::unique_ptr<ISensorSource> src;
    if (mode == "sdk") {
        src = std::make_unique<SdkSource>();
    } else if (mode == "ros") {
#ifdef DEMO_HAS_ROS
        src = std::make_unique<RosSource>(prefix);
#else
        fprintf(stderr,
                "this build has no ROS mode (rclcpp not found at build "
                "time).\nRebuild inside a ROS 2 environment to enable "
                "--mode ros.\n");
        return 2;
#endif
    } else {
        return printUsage();
    }

    GuiWindow window(src.get());
    window.show();
    src->start();
    if (!dumpShot.isEmpty() || !dumpCsv.isEmpty()) {
        QTimer::singleShot(dumpDelaySec * 1000, &app,
                           [&app, &window, &dumpShot, &dumpCsv] {
            if (!dumpShot.isEmpty()) {
                const bool ok = window.grab()
                                    .toImage()
                                    .save(dumpShot.toUtf8().constData());
                fprintf(stderr, ok ? "dumped %s\n" : "dump FAILED\n",
                        dumpShot.toUtf8().constData());
            }
            fprintf(stderr, "counts %s\n",
                    window.frameCounts().toUtf8().constData());
            if (!dumpCsv.isEmpty())
                fprintf(stderr, "dump-csv %s: %d rows\n",
                        dumpCsv.toUtf8().constData(),
                        window.dumpCsv(dumpCsv));
            app.quit();
        });
    }
    const int rc = app.exec();
    src->stop();
    return rc;
}
