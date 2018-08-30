# define platform specific paths in this file
SET(TOOLCHAIN_DIR "/opt" CACHE STRING "")

include(cmake/TargetArch.cmake)
target_architecture(ARCH)
MESSAGE("TOOLCHAIN_DIR is set to:  ${TOOLCHAIN_DIR}")
MESSAGE("BUILDING FOR:  ${ARCH}")

#include target specific linker and include_directories
IF(${ARCH} MATCHES "aarch64")

    set(QT5_DIR ${TOOLCHAIN_DIR}/qt5-odroid)

    set(ZEROMQ_INCLUDE_PATH ${TOOLCHAIN_DIR}/zeromq-odroid/include)
    set(ZEROMQ_LIB_PATH ${TOOLCHAIN_DIR}/zeromq-odroid/lib)

    set(PROTOBUF_INCLUDE_PATH ${TOOLCHAIN_DIR}/protobuf-odroid/include)
    set(PROTOBUF_LIB_PATH ${TOOLCHAIN_DIR}/protobuf-odroid/lib)
    set(PROTOBUF_PROTOC ${TOOLCHAIN_DIR}/protobuf-host/bin/protoc)

    set(LIBSODIUM_LIB_PATH ${TOOLCHAIN_DIR}/libsodium-odroid/lib)

    set(OpenCV_DIR ${TOOLCHAIN_DIR}/opencv-odroid/share/OpenCV/)
ELSEIF(${ARCH} MATCHES "x86_64")
    set(QT5_DIR ${TOOLCHAIN_DIR}/qt5-host)

    set(ZEROMQ_INCLUDE_PATH ${TOOLCHAIN_DIR}/zeromq-host/include)
    set(ZEROMQ_LIB_PATH ${TOOLCHAIN_DIR}/zeromq-host/lib)

    set(PROTOBUF_INCLUDE_PATH ${TOOLCHAIN_DIR}/protobuf-host/include)
    set(PROTOBUF_LIB_PATH ${TOOLCHAIN_DIR}/protobuf-host/lib)
    set(PROTOBUF_PROTOC ${TOOLCHAIN_DIR}/protobuf-host/bin/protoc)

    set(LIBSODIUM_LIB_PATH ${TOOLCHAIN_DIR}/libsodium-host/lib)

    set(OpenCV_DIR ${TOOLCHAIN_DIR}/opencv-host/share/OpenCV/)
ENDIF(${ARCH} MATCHES "aarch64")

# set Qt CMake paths
set(CMAKE_PREFIX_PATH ${QT5_DIR}/lib/cmake)
set(Qt5Core_DIR ${QT5_DIR}/lib/cmake/Qt5Core)
set(Qt5Scxml_DIR ${QT5_DIR}/lib/cmake/Qt5Scxml)
set(Qt5Qml_DIR ${QT5_DIR}/lib/cmake/Qt5Qml)
set(Qt5Network_DIR ${QT5_DIR}/lib/cmake/Qt5Network)
set(Qt5Test_DIR ${QT5_DIR}/lib/cmake/Qt5Test)

MESSAGE("CMAKE_PREFIX_PATH is set to:  ${CMAKE_PREFIX_PATH}")
MESSAGE("Qt5Core_DIR is set to:  ${Qt5Core_DIR}")
MESSAGE("Qt5Scxml_DIR is set to:  ${Qt5Scxml_DIR}")
MESSAGE("Qt5Qml_DIR is set to:  ${Qt5Qml_DIR}")
MESSAGE("Qt5Network_DIR is set to:  ${Qt5Network_DIR}")
MESSAGE("Qt5Test_DIR is set to:  ${Qt5Test_DIR}")
