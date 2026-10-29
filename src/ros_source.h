#pragma once

#include <atomic>
#include <QThread>
#include <QString>
#include "source.h"

class RosSource : public ISensorSource
{
    Q_OBJECT
public:
    explicit RosSource(const QString &topic_prefix, QObject *parent = nullptr)
        : ISensorSource(parent), prefix_(topic_prefix) {}
    ~RosSource() override { stop(); }

    void start() override;
    void stop() override;
    QString describe() const override
    {
        return QStringLiteral("ROS 2 topics (%1/*)").arg(prefix_);
    }

private:
    void run();

    std::atomic<bool> running_{false};
    QThread *thread_ = nullptr;
    QString prefix_;
};
