
cmake_minimum_required(VERSION 3.4.1)


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/android )

# 添加include路径
include_directories(src/lib_tango_client_api/include
                    src/lib_tango_support_api/include
                    src/main/cpp
                    lightweight_filtering/include
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    /opt/android/include )

add_library( sense-data-app
             SHARED
             src/main/cpp/SenseDataApp.cpp
             src/main/jni/jni_interface.cpp )

find_library( log-lib log )

# 添加tango库文件及指定库文件位置,{ANDROID_ABI}变量指定ABI平台
add_library(Tango_LIBRARY SHARED IMPORTED)
set_target_properties( Tango_LIBRARY
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_CURRENT_SOURCE_DIR}/src/lib_tango_client_api/lib/${ANDROID_ABI}/libtango_client_api.so )


add_library(Tango_support_LIBRARY SHARED IMPORTED)
set_target_properties( Tango_support_LIBRARY
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_CURRENT_SOURCE_DIR}/src/lib_tango_support_api/lib/${ANDROID_ABI}/libtango_support_api.so )


# 指定OpenCV库位置并加载
set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)

# 指定Pangolin库的cmake 配置位置，cmake自动查找PangolinConfig.cmake文件并加载
# set(Pangolin_DIR /opt/android/lib/cmake/Pangolin)
# find_package(Pangolin REQUIRED)
# include_directories( ${Pangolin_INCLUDE_DIR} )

# 指定lightweight_filtering库文件夹
set(kindr_DIR /opt/android/share/kindr/cmake)
set( EIGEN3_INCLUDE_DIR /opt/android/include/eigen3 )
add_subdirectory(lightweight_filtering)

# 指定yaml-cpp库
add_library(yaml-cpp STATIC IMPORTED)
set_target_properties( yaml-cpp
                       PROPERTIES IMPORTED_LOCATION
                       /opt/android/lib/libyaml-cpp.a )

# 配置Eigen3头文件
include_directories( /opt/android/include/eigen3 )

# 编译rovio
add_library(rovio
            STATIC
            rovio/Camera.cpp
            rovio/FeatureCoordinates.cpp
            rovio/FeatureDistance.cpp
            # rovio/Scene.cpp 不需要Scene
            )

target_link_libraries( rovio
                       yaml-cpp
                       ${OpenCV_LIBS}
                       )

target_link_libraries( sense-data-app
                       android
                       ${log-lib}
                       Tango_LIBRARY
                       Tango_support_LIBRARY
                       ${OpenCV_LIBS}
                       yaml-cpp
                       rovio
                     )
