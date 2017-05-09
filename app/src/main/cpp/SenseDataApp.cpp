
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
#include <yaml-cpp/yaml.h>
#include <rovio/Camera.hpp>
#include <rovio/RovioFilter.hpp>
#include <thread>
#include <android/asset_manager.h>

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


static int PicNum = 0;

// 点云回调
static void OnPointCloudAvailableRouter(void* context, const TangoPointCloud *point_cloud) {
    LOGI("point cloud nums: %d", point_cloud->num_points);
}

static void OnFrameAvailableRouter(void* context, TangoCameraId cameraId, const TangoImageBuffer *buffer) {
    cv::Mat tangoImage;
    cv::Mat gray;
    if (buffer->format == TANGO_HAL_PIXEL_FORMAT_RGBA_8888) {
        tangoImage = cv::Mat(buffer->height, buffer->width, CV_8UC4, buffer->data);
        cv::cvtColor(tangoImage, gray, CV_BGRA2GRAY);
    }else if(buffer->format == TANGO_HAL_PIXEL_FORMAT_YV12) {
        tangoImage = cv::Mat(buffer->height+buffer->height/2, buffer->width, CV_8UC1, buffer->data);
        cv::cvtColor(tangoImage, gray, CV_YUV2GRAY_YV12);
    }else if(buffer->format == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP) {
        tangoImage = cv::Mat(buffer->height + buffer->height / 2, buffer->width, CV_8UC1, buffer->data);
        cv::cvtColor(tangoImage, gray, CV_YUV2GRAY_NV21);
    }else if(buffer->format == 35){
        tangoImage = cv::Mat(buffer->height+buffer->height/2, buffer->width, CV_8UC1, buffer->data);
        cv::cvtColor(tangoImage, gray, cv::COLOR_YUV420sp2GRAY);
    }else {
        LOGE("Not Support Image Format: %d", buffer->format);
    }

    SenseDataApp *app= static_cast<SenseDataApp*>(context);
    app->onFrameAvailable(buffer, gray);

}

const int my_LOOPER_ID_USER = 3;
int accCounter = 0;
int gyroCounter = 0;
ASensorEventQueue *sensorEventQueue;

// IMU数据回调
static int get_sensor_events(int fd, int events, void* data) {
    ASensorEvent event;
    while (ASensorEventQueue_getEvents(sensorEventQueue, &event, 1) > 0) {
        if(event.type == ASENSOR_TYPE_ACCELEROMETER) {
            // LOGI("accl(x,y,z,t): %f %f %f %ld", event.acceleration.x, event.acceleration.y, event.acceleration.z, event.timestamp);
            if(accCounter == 0 || accCounter == 1000) {
                // LOGI("Acc-Time: %ld (%f)", event.timestamp,((double)(event.timestamp-lastAccTime))/1000000000.0);
                accCounter = 0;
            }
            accCounter++;
        }
        else if(event.type == ASENSOR_TYPE_GYROSCOPE) {
            // LOGI("gyro(axisX,axisY,axisZ,t): %f %f %f %ld", event.uncalibrated_gyro.uncalib[0], event.uncalibrated_gyro.uncalib[1], event.uncalibrated_gyro.uncalib[2], event.timestamp);
            if(gyroCounter == 0 || gyroCounter == 1000) {
                // LOGI("Gyro-Time: %ld (%f)", event.timestamp,((double)(event.timestamp-lastGyroTime))/1000000000.0);
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
    SenseDataApp *app;

public:
    IMUData(){};
    void init(SenseDataApp *a) {
        app = a;
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



/// Tango相机操作
class CameraData {
    TangoConfig tango_config_;
    TangoSupportPointCloudManager *point_cloud_manager_;
    SenseDataApp *app;

public:
    CameraData(){
    };

    ~CameraData() {
        if (tango_config_) {
            TangoConfig_free(tango_config_);
        }
    }

    void onCreate(JNIEnv *env, jobject activity, SenseDataApp *a) {
        app = a;
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
        ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, app, OnFrameAvailableRouter);
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
        TangoPoseData cToIMUPose;
        TangoCoordinateFramePair cToIMUPair;
        cToIMUPair.base = TANGO_COORDINATE_FRAME_IMU;
        cToIMUPair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
        TangoService_getPoseAtTime(0.0, cToIMUPair, &cToIMUPose);
        LOGI("---camera2IMU translation: %lf, %lf, %lf", cToIMUPose.translation[0],
                                   cToIMUPose.translation[1],
                                   cToIMUPose.translation[2]);
        LOGI("----camera2IMU rotation:%lf, %lf, %lf, %lf", cToIMUPose.orientation[3],
                                       cToIMUPose.orientation[0],
                                       cToIMUPose.orientation[1],
                                       cToIMUPose.orientation[2]);

    };

    void onPause() {
        TangoConfig_free(tango_config_);
        tango_config_ = nullptr;
        TangoService_disconnect();
    }
};


// 定义全局数据访问
IMUData gIMUData;
CameraData gCameraData;
// 全局滤波器
std::shared_ptr<mtFilter> mpFilter(new mtFilter);

/* SensorDataApp */
SenseDataApp::SenseDataApp(){
}

SenseDataApp::~SenseDataApp(){
}

typedef typename mtFilter::mtFilterState mtFilterState;
typedef typename mtFilterState::mtState mtState;
typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
mtPredictionMeas predictionMeas_;
typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
typedef typename mtImgUpdate::mtMeas mtImgMeas;
mtImgMeas imgUpdateMeas_;
mtImgUpdate* mpImgUpdate_;
typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
mtPoseMeas poseUpdateMeas_;
mtPoseUpdate* mpPoseUpdate_;
typedef typename std::tuple_element<2,typename mtFilter::mtUpdates>::type mtVelocityUpdate;
typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
mtVelocityMeas velocityUpdateMeas_;


void makeTest(){
    LOGI("-----0");
    mtFilterState* mpTestFilterState = new mtFilterState();
    *mpTestFilterState = mpFilter->init_;
    LOGI("-----1");
    mpTestFilterState->setCamera(&mpFilter->multiCamera_);
    mtState& testState = mpTestFilterState->state_;
    unsigned int s = 2;
    testState.setRandom(s);
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);
    LOGI("-----3");
    delete mpTestFilterState;
}

void SenseDataApp::onCreate(JNIEnv *env, jobject activity) {
    gIMUData.init(this);
    gCameraData.onCreate(env, activity, this);

    gTangoCamera.setTangoCamera();


    // 读取rovio配置文件
    // 初始化滤波器
    mpFilter->readFromInfo("/sdcard/rovio.info");
    mpFilter->refreshProperties();
    LOGI("depth type %d", mpFilter->depthTypeInt_);
//    makeTest();

    // 初始化变量
    is_yuv_texture_available_ = false;
    swap_buffer_signal_ = false;
    is_service_connected_ = false;
    is_texture_id_set_ = false;
    video_overlay_drawable_ = NULL;
    yuv_drawable_ = NULL;
    is_video_overlay_rotation_set_ = false;
    current_texture_method_ = TextureMethod::kYuv;
    isCalibrationMode = false;
    isInCalibration = false;
    shouldCalibration = false;
}

void SenseDataApp::onTangoServiceConnected(JNIEnv *env, jobject binder) {
    gCameraData.onTangoServiceConnected(env, binder);
    is_service_connected_ = true;
}

void SenseDataApp::onResume() {
    gIMUData.onResume();
}
void SenseDataApp::onPause() {
    gIMUData.onPause();
    gCameraData.onPause();

    is_yuv_texture_available_ = false;
    swap_buffer_signal_ = false;
    is_service_connected_ = false;
    is_video_overlay_rotation_set_ = false;
    is_texture_id_set_ = false;
    isInCalibration = false;
    shouldCalibration = false;
    rgb_buffer_.clear();
    yuv_buffer_.clear();
    yuv_temp_buffer_.clear();
    this->DeleteDrawables();
}
void SenseDataApp::DeleteDrawables() {
    delete video_overlay_drawable_;
    delete yuv_drawable_;
    video_overlay_drawable_ = NULL;
    yuv_drawable_ = NULL;

}


// OpenGLES创建
void SenseDataApp::onGLSurfaceCreated(){
    if (video_overlay_drawable_ != NULL || yuv_drawable_ != NULL) {
        this->DeleteDrawables();
    }

    video_overlay_drawable_ = new tango_gl::VideoOverlay(GL_TEXTURE_EXTERNAL_OES, display_rotation_);
    yuv_drawable_ = new tango_gl::VideoOverlay(GL_TEXTURE_2D, display_rotation_);
}
void SenseDataApp::onGLSurfaceChanged(int width, int height){
    glViewport(0, 0, width, height);
}
// 主渲染循环
// 标定
void SenseDataApp::myCalibrateCamera() {
    using namespace std;
    using namespace cv;
    Mat corners;
    Size boardSize = Size(11, 8);

    clock_t start, end;
    start = clock();
    int r = findChessboardCorners(grayImage, boardSize, corners);
    end = clock();
    LOGE("find chessboard corners time: %lf", (double)(end-start)/CLOCKS_PER_SEC);
    if (r == 1) {
        std::ostringstream ss;
        ss << "/sdcard/tmp/" << PicNum << ".png";
        cv::imwrite(ss.str(), grayImage);
        PicNum ++;
    }
    std::lock_guard<std::mutex> lock(calibration_mutex);
    isInCalibration = false;
}
void SenseDataApp::render() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    if (!is_service_connected_) {
        return;
    }
    if (shouldCalibration) {
        {
            std::lock_guard<std::mutex> lock(calibration_mutex);
            shouldCalibration = false;
        }
        std::thread t(&SenseDataApp::myCalibrateCamera, this);
        t.detach();
    }

    if (!is_texture_id_set_) {
        is_texture_id_set_ = true;
        // Connect color camera texture. TangoService_connectTextureId expects a
        // valid texture id from the caller, so we will need to wait until the GL
        // content is properly allocated.
        int texture_id = static_cast<int>(video_overlay_drawable_->GetTextureId());
        TangoErrorType ret = TangoService_connectTextureId(TANGO_CAMERA_COLOR, texture_id, nullptr, nullptr);
        if (ret != TANGO_SUCCESS) {
            LOGE("SenseDataApp: Failed to connect the texture id with error" "code: %d", ret);
        }
    }

    if (!is_video_overlay_rotation_set_) {
        video_overlay_drawable_->SetDisplayRotation(display_rotation_);
        yuv_drawable_->SetDisplayRotation(display_rotation_);
        is_video_overlay_rotation_set_ = true;
    }

    switch (current_texture_method_) {
        case TextureMethod::kYuv:
            RenderYuv();
            break;
        case TextureMethod::kTextureId:
            RenderTextureId();
            break;
    }
}

void SenseDataApp::AllocateTexture(GLuint texture_id, int width, int height) {
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, rgb_buffer_.data());
}

inline void Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r,
                    uint8_t* g, uint8_t* b) {
    float R = yValue + 1.402 * (vValue - 128) ;
    float G = yValue - 0.344 * (uValue - 128) - 0.714 * (vValue - 128);
    float B = yValue + 1.772 * (uValue - 128);

    R= R * !(R<0);
    G= G * !(G<0);
    B= B * !(B<0);

    *r = R*(!(R>255)) + 255 * (R>255);
    *g = G*(!(G>255)) + 255 * (G>255);
    *b = B*(!(B>255)) + 255 * (B>255);
}

void SenseDataApp::RenderYuv() {
    if (!is_yuv_texture_available_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
        if (swap_buffer_signal_) {
            std::swap(yuv_buffer_, yuv_temp_buffer_);
            swap_buffer_signal_ = false;
        }
    }

    for (size_t i = 0; i < yuv_height_; ++i) {
        for (size_t j = 0; j < yuv_width_; ++j) {
            size_t x_index = j;
            if (j % 2 != 0) {
                x_index = j - 1;
            }

            size_t rgb_index = (i * yuv_width_ + j) * 3;

            // The YUV texture format is NV21,
            // yuv_buffer_ buffer layout:
            //   [y0, y1, y2, ..., yn, v0, u0, v1, u1, ..., v(n/4), u(n/4)]
            Yuv2Rgb(
                yuv_buffer_[i * yuv_width_ + j],
                yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index + 1],
                yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index],
                &rgb_buffer_[rgb_index], &rgb_buffer_[rgb_index + 1],
                &rgb_buffer_[rgb_index + 2]);
        }
    }

    glBindTexture(GL_TEXTURE_2D, yuv_drawable_->GetTextureId());
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, yuv_width_, yuv_height_, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, rgb_buffer_.data());

    yuv_drawable_->Render(glm::mat4(1.0f), glm::mat4(1.0f));
}

void SenseDataApp::onFrameAvailable(const TangoImageBuffer *buffer, cv::Mat gray){

    if (!gray.empty()) {
        // 获取相机标定
        if(!isInCalibration && isCalibrationMode) {
            grayImage = gray.clone();
            {
                std::lock_guard<std::mutex> lock(calibration_mutex);
                isInCalibration = true;
                shouldCalibration = true;
            }
        }
    }
    if (current_texture_method_ != TextureMethod::kYuv) {
        return;
    }
    if (yuv_drawable_ == NULL) {
        return;
    }
    if (yuv_drawable_->GetTextureId() == 0) {
        LOGE("HelloVideoApp::yuv texture id not valid");
        return;
    }
    if (buffer->format != TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP) {
        LOGE("HelloVideoApp::yuv texture format is not supported by this app");
        return;
    }
    // 根据图像大小分配内存
    if (!is_yuv_texture_available_) {
        yuv_width_ = buffer->width;
        yuv_height_ = buffer->height;
        uv_buffer_offset_ = yuv_width_ * yuv_height_;

        yuv_size_ = yuv_width_ * yuv_height_ + yuv_width_ * yuv_height_ / 2;

        // Reserve and resize the buffer size for RGB and YUV data.
        yuv_buffer_.resize(yuv_size_);
        yuv_temp_buffer_.resize(yuv_size_);
        rgb_buffer_.resize(yuv_width_ * yuv_height_ * 3);

        AllocateTexture(yuv_drawable_->GetTextureId(), yuv_width_, yuv_height_);
        is_yuv_texture_available_ = true;
    }

    std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
    memcpy(&yuv_temp_buffer_[0], buffer->data, yuv_size_);
    swap_buffer_signal_ = true;
}
void SenseDataApp::RenderTextureId() {
    video_overlay_drawable_->Render(glm::mat4(1.0f), glm::mat4(1.0f));
}

void SenseDataApp::OnDisplayChanged(int display_rotation) {
    display_rotation_ = static_cast<TangoSupportRotation>(display_rotation);
    is_video_overlay_rotation_set_ = false;
}

