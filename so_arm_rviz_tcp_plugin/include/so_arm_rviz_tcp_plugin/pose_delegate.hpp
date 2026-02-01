# pragma once

#include <QStyledItemDelegate>
#include <QLineEdit>

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
};