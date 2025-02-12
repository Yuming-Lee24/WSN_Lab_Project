#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDateTime>
#include <QApplication>
#include <QPushButton>
#include <QLayout>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QString>
#include <QByteArray>
#include <QTimer>
#include <QComboBox>
#include <QTextEdit>
#include "qextserialport.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void receive();
    void on_pushButton_Open_clicked();
    void on_pushButton_Close_clicked();
    void clearOldTableEntries();

private:
    Ui::MainWindow *ui;
    QextSerialPort *port;
    QByteArray buffer;
    QTimer *cleanupTimer;
};
#endif // MAINWINDOW_H
