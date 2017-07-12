#-------------------------------------------------
#
# Project created by QtCreator 2017-06-21T12:54:30
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TouchWidget
TEMPLATE = app



# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
#### OpenCV


SOURCES += \
        main.cpp \
        touchwidget.cpp \
    PracticalSocket.cpp

HEADERS += \
        touchwidget.h \
    config.h \
    PracticalSocket.h

FORMS += \
        touchwidget.ui

CONFIG += mobility
MOBILITY = 

#INCLUDEPATH += /usr/local/include
#LIBS += -L/usr/local/lib
#LIBS += `pkg-config opencv --libs`

linux:!android {
    # using pkg-config
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

android {
    # full path to OpenCV Android SDK
    OPENCV_PATH = "$$_PRO_FILE_PWD_/OpenCV-2.4.10-android-sdk"

    INCLUDEPATH += $${OPENCV_PATH}/sdk/native/jni/include/

    LIBS += -L$${OPENCV_PATH}/sdk/native/libs/armeabi-v7a/ \
            -lopencv_androidcamera \
            -lopencv_calib3d \
            -lopencv_contrib \
            -lopencv_core \
            -lopencv_features2d \
            -lopencv_flann \
            -lopencv_highgui \
            -lopencv_imgproc \
            -lopencv_legacy \
            -lopencv_ml \
            -lopencv_objdetect \
            -lopencv_photo \
            -lopencv_stitching \
            -lopencv_ts \
            -lopencv_video \
            -lopencv_videostab

    LIBS += -L$${OPENCV_PATH}/sdk/native/3rdparty/libs/armeabi-v7a \
            -lIlmImf \
            -llibjasper \
            -llibjpeg \
            -llibpng \
            -llibtiff \
            -ltbb
}

#DISTFILES += \
#    android/AndroidManifest.xml
ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android-sources

DISTFILES += \
    android-sources/AndroidManifest.xml
