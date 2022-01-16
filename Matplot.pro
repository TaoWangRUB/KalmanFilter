TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        kalman_constant.cpp \
        kalman_linear.cpp \
        main.cpp

INCLUDEPATH += /home/qjn35x/install/package/python/3.8.2/include/python3.8
INCLUDEPATH += /home/qjn35x/install/package/python/3.8.2/lib/python3.8/site-packages/numpy/core/include
LIBS += -L/home/qjn35x/install/package/python/3.8.2/lib -lpython3.8

HEADERS += \
    kalman_constant.h \
    kalman_linear.h \
    matplotlibcpp.h
