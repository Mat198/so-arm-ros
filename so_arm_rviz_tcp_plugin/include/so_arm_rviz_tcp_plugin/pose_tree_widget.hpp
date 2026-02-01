#pragma once

#include <QTreeWidget>
#include <QDropEvent>

class PoseTreeWidget : public QTreeWidget {
    Q_OBJECT

public:
    explicit PoseTreeWidget(QWidget *parent = nullptr);

protected:
    // Overriding the drop event
    void dropEvent(QDropEvent *event) override;

signals:
    void itemDropped(); // Custom signal to notify other parts of your app
};