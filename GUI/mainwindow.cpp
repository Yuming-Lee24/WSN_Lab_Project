#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QRegExp>
#include <QDebug>
#include "qextserialport.h"
#include "qextserialenumerator.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    qDebug() << "MainWindow constructor started";
    ui->setupUi(this);

    // List available ports
    QList<QextPortInfo> ports = QextSerialEnumerator::getPorts();
    for (const QextPortInfo &info : ports) {
        ui->comboBox_Port->addItem(info.portName);
    }

    // Setup serial port
    port = new QextSerialPort();
    port->setBaudRate(BAUD115200);
    port->setDataBits(DATA_8);
    port->setParity(PAR_NONE);
    port->setStopBits(STOP_1);
    port->setFlowControl(FLOW_OFF);
    // Setup QTextEdit
    ui->textEdit_Status->setAcceptRichText(true);
    ui->textEdit_Status->document()->setDefaultStyleSheet("span { color: black; }");
    // Setup table
    ui->tableWidget_Routing->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget_Routing->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // Setup cleanup timer
    cleanupTimer = new QTimer(this);
    cleanupTimer->setInterval(5000); // Check every 5 seconds
    connect(cleanupTimer, &QTimer::timeout, this, &MainWindow::clearOldTableEntries);
    cleanupTimer->start();

    connect(port, SIGNAL(readyRead()), this, SLOT(receive()));
}

MainWindow::~MainWindow()
{
    if(port->isOpen()) {
        port->close();
    }
    delete port;
    delete ui;
}

void MainWindow::clearOldTableEntries()
{
    QDateTime currentTime = QDateTime::currentDateTime();

    for(int row = ui->tableWidget_Routing->rowCount() - 1; row >= 0; --row) {
        QTableWidgetItem *lastSeenItem = ui->tableWidget_Routing->item(row, 2);
        if(lastSeenItem) {
            int lastSeen = lastSeenItem->text().toInt();
            if(lastSeen > 30) { // Remove entries not seen for more than 30 seconds
                ui->tableWidget_Routing->removeRow(row);
            }
        }
    }
}

void MainWindow::on_pushButton_Open_clicked()
{
    QString portName = ui->comboBox_Port->currentText();
    port->setPortName(portName);

    if(port->open(QIODevice::ReadWrite)) {
        QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss");
        ui->textEdit_Status->append(QString("[%1] Serial port %2 opened successfully\n")
                                  .arg(timeStr)
                                  .arg(portName));
        ui->pushButton_Open->setEnabled(false);
        ui->pushButton_Close->setEnabled(true);
        ui->comboBox_Port->setEnabled(false);
    } else {
        QMessageBox::critical(this, "Error",
                            QString("Failed to open serial port %1!").arg(portName));
    }
}

void MainWindow::on_pushButton_Close_clicked()
{
    if(port->isOpen()) {
        QString portName = port->portName();
        port->close();
        QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss");
        ui->textEdit_Status->append(QString("[%1] Serial port %2 closed")
                                  .arg(timeStr)
                                  .arg(portName));
        ui->pushButton_Open->setEnabled(true);
        ui->pushButton_Close->setEnabled(false);
        ui->comboBox_Port->setEnabled(true);
    }
}
void MainWindow::receive()
{
    QByteArray newData = port->readAll();
    buffer.append(newData);
    QString data = QString::fromUtf8(buffer);

    // Extract and process complete routing table
    QRegExp rtStartRegex("╔═+\\s*NODE\\s+\\d+\\s+ROUTING\\s+TABLE");
    QRegExp rtEndRegex("╚═+╝");
    int startPos = rtStartRegex.indexIn(data);
    int endPos = rtEndRegex.lastIndexIn(data);

    // Process emergency messages
    QRegExp emergencyRegex("╔═+\\s*EMERGENCY MESSAGE[^╚]*╚═+╝");
    int emergencyPos = 0;
    while ((emergencyPos = emergencyRegex.indexIn(data, emergencyPos)) != -1) {
        QString emergencyMsg = emergencyRegex.cap(0);

        QRegExp sourceRegex("Source: Node\\s+(\\d+)");
        QRegExp tempRegex("Temperature:\\s+([-]?\\d+\\.\\d+)");

        QString nodeId = sourceRegex.indexIn(emergencyMsg) != -1 ? sourceRegex.cap(1) : "";
        QString temperature = tempRegex.indexIn(emergencyMsg) != -1 ? tempRegex.cap(1) : "";

        if (!nodeId.isEmpty() && !temperature.isEmpty()) {
            QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss");
            QString message = QString("[%1] <span style=\"color: red;\">EMERGENCY: Node %2 Temperature: %3°C</span>")
                                .arg(timeStr)
                                .arg(nodeId)
                                .arg(temperature);
            ui->textEdit_Status->append(message);
        }

        emergencyPos += emergencyRegex.matchedLength();
    }

    if (startPos != -1 && endPos != -1 && endPos > startPos) {
        QString routingTable = data.mid(startPos, endPos - startPos + rtEndRegex.matchedLength());
        qDebug() << "Processing routing table:" << routingTable;

        QRegExp nodeLineRegex("║\\s*(\\d+)\\s*\\|\\s*(-?\\d+)\\s*\\|\\s*(\\d+)\\s*\\|\\s*(\\w+)\\s*\\|\\s*(\\w+)\\s*║");
        int pos = 0;

        ui->tableWidget_Routing->setRowCount(0);

        while ((pos = nodeLineRegex.indexIn(routingTable, pos)) != -1) {
            int row = ui->tableWidget_Routing->rowCount();
            ui->tableWidget_Routing->insertRow(row);

            for (int i = 1; i <= 5; i++) {
                QString value = nodeLineRegex.cap(i).trimmed();
                ui->tableWidget_Routing->setItem(row, i-1, new QTableWidgetItem(value));
            }
            pos += nodeLineRegex.matchedLength();
        }
    }

    // Process data packets for sensor data
    QRegExp packetRegex("╔═+\\s*RECEIVED DATA[^╚]*╚═+╝");
    int pos = 0;
    while ((pos = packetRegex.indexIn(data, pos)) != -1) {
        QString packet = packetRegex.cap(0);

        QRegExp sourceRegex("Source: Node\\s+(\\d+)");
        QRegExp tempRegex("Temperature:\\s+([-]?\\d+\\.\\d+)");
        QRegExp statusRegex("Status:\\s+(\\w+)");
        QRegExp pathRegex("Path:\\s+([0-9 ->]+)");

        QString nodeId = sourceRegex.indexIn(packet) != -1 ? sourceRegex.cap(1) : "";
        QString temperature = tempRegex.indexIn(packet) != -1 ? tempRegex.cap(1) : "";
        QString status = statusRegex.indexIn(packet) != -1 ? statusRegex.cap(1) : "";
        QString path = pathRegex.indexIn(packet) != -1 ? pathRegex.cap(1) + " -> gateway" : "";

        if (!nodeId.isEmpty()) {
            // Update status window
            QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss");
            ui->textEdit_Status->append(QString("[%1] Info: Received data from node %2, Path: %3")
                                      .arg(timeStr).arg(nodeId).arg(path));

            // Update sensor data table
            bool found = false;
            for(int row = 0; row < ui->tableWidget_SensorData->rowCount(); ++row) {
                if(ui->tableWidget_SensorData->item(row, 0)->text() == nodeId) {
                    ui->tableWidget_SensorData->item(row, 1)->setText(temperature);
                    ui->tableWidget_SensorData->item(row, 2)->setText(status);
                    found = true;
                    break;
                }
            }

            if(!found) {
                int row = ui->tableWidget_SensorData->rowCount();
                ui->tableWidget_SensorData->insertRow(row);
                ui->tableWidget_SensorData->setItem(row, 0, new QTableWidgetItem(nodeId));
                ui->tableWidget_SensorData->setItem(row, 1, new QTableWidgetItem(temperature));
                ui->tableWidget_SensorData->setItem(row, 2, new QTableWidgetItem(status));
            }
        }

        pos += packetRegex.matchedLength();
    }

    if (endPos != -1 || packetRegex.indexIn(data) != -1 || emergencyRegex.indexIn(data) != -1) {
        buffer.clear();
    } else if (buffer.size() > 8192) {
        buffer.clear();
    }
}
