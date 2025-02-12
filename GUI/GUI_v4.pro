QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += debug
TEMPLATE = app
TARGET = wsn_monitor

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    mainwindow.h

FORMS += \
    mainwindow.ui

HOMEDIR = $$(HOME)
include($$HOMEDIR/qextserialport/src/qextserialport.pri)
