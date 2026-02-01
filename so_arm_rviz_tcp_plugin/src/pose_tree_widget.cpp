#include "so_arm_rviz_tcp_plugin/pose_tree_widget.hpp"

PoseTreeWidget::PoseTreeWidget(QWidget *parent) : QTreeWidget(parent) {
    
    this->setHeaderHidden(true);
    this->setSelectionMode(QAbstractItemView::SingleSelection);
    this->setColumnCount(2);
    this->setAlternatingRowColors(true);
    this->setExpandsOnDoubleClick(true);
    this->setDragDropMode(QAbstractItemView::InternalMove);
    this->setDefaultDropAction(Qt::MoveAction);
    this->setDragEnabled(true);
    this->setAcceptDrops(true);
    this->setDropIndicatorShown(true);
}

void PoseTreeWidget::dropEvent(QDropEvent *event) {
    QTreeWidget::dropEvent(event);
    if (event->isAccepted()) {
        // Send custom signal to organize the pose list
        emit itemDropped();
    }
}