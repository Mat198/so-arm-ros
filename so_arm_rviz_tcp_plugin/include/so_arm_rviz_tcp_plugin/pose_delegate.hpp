#pragma once

#include <QStyledItemDelegate>
#include <QLineEdit>
#include <QPushButton>
#include <QApplication>
#include <QMouseEvent>

class PoseDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    using QStyledItemDelegate::QStyledItemDelegate;

    QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& /* option */, 
        const QModelIndex& index
    ) const override {
        
        // Editable second column for the children an editable first column for the top level.
        if (index.parent().isValid()) {
            if (index.column() == 1) return new QLineEdit(parent);
        } else {
            if (index.column() == 0) return new QLineEdit(parent);
        }
        return nullptr;
    }

    // Transfer data from Model -> Editor
    void setEditorData(QWidget* editor, const QModelIndex& index) const override {
        
        QString value = index.model()->data(index, Qt::EditRole).toString();
        QLineEdit* lineEdit = static_cast<QLineEdit*>(editor);
        lineEdit->setText(value);
    }

    // Transfer data from Editor -> Model
    void setModelData(
        QWidget* editor, QAbstractItemModel* model, const QModelIndex& index
    ) const override {

        QLineEdit* lineEdit = static_cast<QLineEdit*>(editor);
        model->setData(index, lineEdit->text(), Qt::EditRole);
    }

    QRect getButtonRect(const QStyleOptionViewItem &option) const {
        QStyleOptionButton button;
        button.text = "Delete";
        QSize textSize = option.fontMetrics.size(Qt::TextSingleLine, button.text);
        
        // Maintain your specific sizing logic
        QSize fullSize = qApp->style()->sizeFromContents(
            QStyle::CT_PushButton, &button, 
            QSize(textSize.width(), textSize.height() - 10), option.widget);
            
        return QStyle::alignedRect(
            option.direction, Qt::AlignRight | Qt::AlignVCenter, fullSize, option.rect);
    }

    // Draw the delete button on parent in column 1
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
        if (index.parent().isValid() || index.column() != 1) {
            QStyledItemDelegate::paint(painter, option, index);
            return;
        }

        QStyleOptionButton button;
        button.rect = getButtonRect(option);
        button.text = "Delete";
        button.state = QStyle::State_Enabled;

        // Provide the "pushed down" effect if the index is being pressed
        if (option.state & QStyle::State_Sunken) {
            button.state |= QStyle::State_Sunken;
        }

        QApplication::style()->drawControl(QStyle::CE_PushButton, &button, painter);
    }

    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override {
        if (index.parent().isValid() || index.column() != 1) {
            return false;
        }

        QRect buttonRect = getButtonRect(option);
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);

        // 1. Check if the click is actually inside the button, not just the cell
        if (!buttonRect.contains(mouseEvent->pos())) {
            return false;
        }

        if (event->type() == QEvent::MouseButtonPress) {
            // Forces the view to repaint the item with the State_Sunken flag
            return true; 
        }

        if (event->type() == QEvent::MouseButtonRelease) {
            model->removeRow(index.row(), index.parent());
            return true;
        }

        return false;
    }
};