
//
// Created by JinJay on 2017/4/17.
//

#include "SenseDataApp.h"

#include <tango_support_api.h>
#include <android/sensor.h>
#include <android/log.h>
#include <string>
#include <memory>
#include <iostream>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rovio/Camera.hpp>
#include <rovio/RovioFilter.hpp>


#define  LOG_TAG    "SenseDataApp"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

rovio::Camera gTangoCamera;
typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;



// 点云回调
static void OnPointCloudAvailableRouter(void* context, const TangoPointCloud *point_cloud) {
    LOGI("point cloud nums: %d", point_cloud->num_points);
}
static void OnFrameAvailableRouter(void* context, TangoCameraId cameraId, const TangoImageBuffer *buffer) {

    cv::Mat tangoImage;
    if (buffer->format == TANGO_HAL_PIXEL_FORMAT_RGBA_8888) {
        tangoImage = cv::Mat(buffer->height, buffer->width, CV_8UC4, buffer->data);
    }else if(buffer->format == TANGO_HAL_PIXEL_FORMAT_YV12) {
        tangoImage = cv::Mat(buffer->height+buffer->height/2, buffer->width, CV_8UC1, buffer->data);
    }else if(buffer->format == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP) {
        tangoImage = cv::Mat(buffer->height + buffer->height / 2, buffer->width, CV_8UC1, buffer->data);
    }else if(buffer->format == 35){
        tangoImage = cv::Mat(buffer->height+buffer->height/2, buffer->width, CV_8UC1, buffer->data);
    }else {
        LOGE("Not Support Image Format: %d", buffer->format);
    }
    if(!tangoImage.empty()){
        LOGI("r, c: %d, %d, timestamp: %f", tangoImage.rows, tangoImage.cols, buffer->timestamp); // 1080 * 1280
    }
}

const int my_LOOPER_ID_USER = 3;
int accCounter = 0;
int gyroCounter = 0;
int64_t lastAccTime = 0;
int64_t lastGyroTime = 0;
ASensorEventQueue *sensorEventQueue;

// IMU数据回调
static int get_sensor_events(int fd, int events, void* data) {
    ASensorEvent event;
    while (ASensorEventQueue_getEvents(sensorEventQueue, &event, 1) > 0) {
        if(event.type == ASENSOR_TYPE_ACCELEROMETER) {
            LOGI("accl(x,y,z,t): %f %f %f %ld", event.acceleration.x, event.acceleration.y, event.acceleration.z, event.timestamp);
            if(accCounter == 0 || accCounter == 1000) {
                LOGI("Acc-Time: %ld (%f)", event.timestamp,((double)(event.timestamp-lastAccTime))/1000000000.0);
                lastAccTime = event.timestamp;
                accCounter = 0;
            }
            accCounter++;
        }
        else if(event.type == ASENSOR_TYPE_GYROSCOPE) {
            LOGI("gyro(axisX,axisY,axisZ,t): %f %f %f %ld", event.uncalibrated_gyro.uncalib[0], event.uncalibrated_gyro.uncalib[1], event.uncalibrated_gyro.uncalib[2], event.timestamp);
            if(gyroCounter == 0 || gyroCounter == 1000) {
                LOGI("Gyro-Time: %ld (%f)", event.timestamp,((double)(event.timestamp-lastGyroTime))/1000000000.0);
                lastGyroTime = event.timestamp;
                gyroCounter = 0;
            }
            gyroCounter++;
        }
    }
    // 返回1继续接收数据
    return 1;
}

/// IMU数据类
class IMUData {
    ASensorManager *sensorManager;
    const ASensor *accelerometer;
    const ASensor *gyroscope;
    ALooper *looper;

public:
    IMUData(){};
    void init() {
        // 初始化传感器
        sensorManager = ASensorManager_getInstance();
        assert(sensorManager != NULL);
        accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
        assert(accelerometer != NULL);
        gyroscope = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE);
        assert(gyroscope != NULL);

        LOGI("init Sensor Data");

        void* sensor_data = malloc(1000);
        // 定义处理回调
        looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
        assert(looper != NULL);
        sensorEventQueue = ASensorManager_createEventQueue(sensorManager, looper, my_LOOPER_ID_USER, get_sensor_events, sensor_data);

        assert(sensorEventQueue != NULL);
        auto status = ASensorEventQueue_enableSensor(sensorEventQueue, accelerometer);
        assert(status >= 0);
        status = ASensorEventQueue_enableSensor(sensorEventQueue, gyroscope);
        assert(status >= 0);

        status = ASensorEventQueue_setEventRate(sensorEventQueue, accelerometer, ASensor_getMinDelay(accelerometer));
        assert(status >= 0);
        status = ASensorEventQueue_setEventRate(sensorEventQueue, gyroscope, ASensor_getMinDelay(gyroscope));
        assert(status >= 0);
    }

    void onPause() {
        ASensorEventQueue_disableSensor(sensorEventQueue, accelerometer);
        ASensorEventQueue_disableSensor(sensorEventQueue, gyroscope);
    }

    void onResume() {
        ASensorEventQueue_enableSensor(sensorEventQueue, accelerometer);
        ASensorEventQueue_enableSensor(sensorEventQueue, gyroscope);
        auto status = ASensorEventQueue_setEventRate(sensorEventQueue, accelerometer, ASensor_getMinDelay(accelerometer));
        assert(status >= 0);
        status = ASensorEventQueue_setEventRate(sensorEventQueue, gyroscope, ASensor_getMinDelay(gyroscope));
        assert(status >= 0);
    }
};


namespace {
    constexpr int kTangoCoreMinimumVersion = 9377;
}

class CameraData {
    bool is_service_connected_ = false;
    TangoConfig tango_config_;
    TangoSupportPointCloudManager *point_cloud_manager_;

public:
    CameraData() : is_service_connected_(false) {
    };

    ~CameraData() {
        if (tango_config_) {
            TangoConfig_free(tango_config_);
        }
    }

    void onCreate(JNIEnv *env, jobject activity) {
        int version;
        TangoErrorType err = TangoSupport_GetTangoVersion(env, activity, &version);
        if (err != TANGO_SUCCESS || version < kTangoCoreMinimumVersion) {
            LOGE("SynchronizationApplication::OnCreate, Tango Core version is out of date.");
            std::terminate();
        }
    }

    void onTangoServiceConnected(JNIEnv *env, jobject binder) {
        TangoErrorType ret = TangoService_setBinder(env, binder);
        if (ret != TANGO_SUCCESS) {
            LOGE("SynchronizationApplication: Failed to set Tango service binder with error code: %d", ret);
            std::terminate();
        }

        // 初始化Tango支持库
        TangoSupport_initializeLibrary();

        // 初始化Tango库
        TangoSetupConfig();
        TangoConnectCallbacks();
        TangoConnect();
        TangoSetIntrinsics();


        is_service_connected_ = true;
    }

    void TangoSetupConfig() {
        if (tango_config_ != nullptr) {
            return;
        }
        tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
        if (tango_config_ == nullptr) {
            std::terminate();
        }
        // 开启深度
        TangoErrorType err = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
        if (err != TANGO_SUCCESS) {
            LOGE("Failed to enable depth.");
            std::terminate();
        }
        // 指定深度模式
        err = TangoConfig_setInt32(tango_config_, "config_depth_mode", TANGO_POINTCLOUD_XYZC);
        if (err != TANGO_SUCCESS) {
            LOGE("Failed to set 'depth_mode' configuration flag with error code: %d", err);
            std::terminate();
        }
        // 开启彩色相机
        err = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
        if (err != TANGO_SUCCESS) {
            LOGE("Failed to set 'enable_color_camera' configuration flag with error code: %d", err);
            std::terminate();
        }
        // 关闭运动追踪
        err = TangoConfig_setBool(tango_config_, "config_enable_motion_tracking", true);
        if (err != TANGO_SUCCESS) {
            LOGE("Failed to disable motion tracking.");
            std::terminate();
        }
        // 设置回点云函数的属性
        if (point_cloud_manager_ == nullptr) {
            // 点云最大元素数量
            int32_t max_point_cloud_elements;
            err = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                                       &max_point_cloud_elements);
            if (err != TANGO_SUCCESS) {
                LOGE("Failed to query maximum number of point cloud elements.");
                std::terminate();
            }
            err = TangoSupport_createPointCloudManager(max_point_cloud_elements, &point_cloud_manager_);
            if (err != TANGO_SUCCESS) {
                std::terminate();
            }
        }
    }

    void TangoConnect() {
        TangoErrorType ret = TangoService_connect(this, tango_config_);
        if (ret != TANGO_SUCCESS) {
            LOGE("SenseDataApp: Failed to connect to the Tango service.");
            std::terminate();
        }
        LOGI("SenseDataApp: ------connect tango-----");
    }

    void TangoConnectCallbacks() {
        // 定义深度点云回调函数
        TangoErrorType ret = TangoService_connectOnPointCloudAvailable(OnPointCloudAvailableRouter);
        if (ret != TANGO_SUCCESS) {
            LOGE("SenseDataApp: Failed to connect point cloud callback with errorcode: %d", ret);
            std::terminate();
        }
        // 彩色图像回调
        ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, OnFrameAvailableRouter);
        if (ret != TANGO_SUCCESS) {
            LOGE("SenseDataApp: Failed to connect texture callback with errorcode: %d", ret);
            std::terminate();
        }
    };

    void TangoSetIntrinsics() {
        TangoCameraIntrinsics ci;
        TangoErrorType err = TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ci);
        if (err != TANGO_SUCCESS) {
            LOGE("SenseDataApp: Failed to get the intrinsics for the color camera.");
            std::terminate();
        }
        if (ci.calibration_type == TANGO_CALIBRATION_EQUIDISTANT){
            LOGI("Calibration Type: equidistant");
            LOGI("Calibration paras: cx:%lf, cy:%lf, fx:%lf, fy: %lf, h:%u, w: %u, k0: %lf, k1: %lf, k2:%lf, k3:%lf, k4:%lf",
                 ci.cx, ci.cy, ci.fx, ci.fy, ci.height, ci.width, ci.distortion[0], ci.distortion[1], ci.distortion[2], ci.distortion[3], ci.distortion[4]);
        }else if (ci.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_2_PARAMETERS) {
            LOGI("Calibration Type: polynomial 2 parameters");
            LOGI("Calibration paras: cx:%lf, cy:%lf, fx:%lf, fy: %lf, h:%u, w: %u, k0: %lf, k1: %lf, k2:%lf, k3:%lf, k4:%lf",
                 ci.cx, ci.cy, ci.fx, ci.fy, ci.height, ci.width, ci.distortion[0], ci.distortion[1], ci.distortion[2], ci.distortion[3], ci.distortion[4]);
        }else if (ci.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_3_PARAMETERS) {
            LOGI("Calibration Type: polynomial 3 parameters");
            LOGI("Calibration paras: cx:%lf, cy:%lf, fx:%lf, fy: %lf, h:%u, w: %u, k0: %lf, k1: %lf, k2:%lf, k3:%lf, k4:%lf",
                 ci.cx, ci.cy, ci.fx, ci.fy, ci.height, ci.width, ci.distortion[0], ci.distortion[1], ci.distortion[2], ci.distortion[3], ci.distortion[4]);
        }else if (ci.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_5_PARAMETERS) {
            LOGI("Calibration Type: polynomial 5 parameters");
            LOGI("Calibration paras: cx:%lf, cy:%lf, fx:%lf, fy: %lf, h:%u, w: %u, k0: %lf, k1: %lf, k2:%lf, k3:%lf, k4:%lf",
                 ci.cx, ci.cy, ci.fx, ci.fy, ci.height, ci.width, ci.distortion[0], ci.distortion[1], ci.distortion[2], ci.distortion[3], ci.distortion[4]);
        } else {
            LOGI("Calibration Type: unkown");
            LOGI("Calibration paras: cx:%lf, cy:%lf, fx:%lf, fy: %lf, h:%u, w: %u, k0: %lf, k1: %lf, k2:%lf, k3:%lf, k4:%lf",
                 ci.cx, ci.cy, ci.fx, ci.fy, ci.height, ci.width, ci.distortion[0], ci.distortion[1], ci.distortion[2], ci.distortion[3], ci.distortion[4]);
        }
        gTangoCamera.setTangoCamera();
    };

    void onPause() {
        is_service_connected_ = false;
        TangoConfig_free(tango_config_);
        tango_config_ = nullptr;
        TangoService_disconnect();
    }
};

IMUData gIMUData;
CameraData gCameraData;

SenseDataApp::SenseDataApp(){
}

SenseDataApp::~SenseDataApp(){
}

void setImageData(unsigned char * imageArray, int size){
    for(int i = 0 ; i < size;i++) {
        imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
    }
}


/// SensorDataApp
void SenseDataApp::onCreate(JNIEnv *env, jobject activity) {
    gIMUData.init();
    gCameraData.onCreate(env, activity);

}

void SenseDataApp::onTangoServiceConnected(JNIEnv *env, jobject binder) {
    gCameraData.onTangoServiceConnected(env, binder);
}

void SenseDataApp::onResume() {
    gIMUData.onResume();
}
void SenseDataApp::onPause() {
    gIMUData.onPause();
    gCameraData.onPause();
}
