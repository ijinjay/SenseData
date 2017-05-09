//
// Created by JinJay on 2017/4/17.
//

#ifndef SENSEDATA_SENSEDATAAPP_H
#define SENSEDATA_SENSEDATAAPP_H

#include <jni.h>
#include <GLES2/gl2.h>
#include <tango-gl/video_overlay.h>
#include <vector>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>


#define  LOG_TAG    "SenseDataApp"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

/// 应用程序类
class SenseDataApp {
public:
    // 纹理类型
    enum TextureMethod {
        kYuv,
        kTextureId
    };
    SenseDataApp();
    ~SenseDataApp();
    void onCreate(JNIEnv* env, jobject activity);
    void onTangoServiceConnected(JNIEnv* env, jobject activity);
    void onResume();
    void onFrameAvailable(const TangoImageBuffer *buffer, cv::Mat gray);
    void onPause();
    void onGLSurfaceCreated();
    void onGLSurfaceChanged(int width, int height);
    void render();
    void OnDisplayChanged(int display_rotation);

private:

    // 视频纹理渲染层
    tango_gl::VideoOverlay* video_overlay_drawable_;
    tango_gl::VideoOverlay* yuv_drawable_;

    TextureMethod current_texture_method_;

    std::vector<uint8_t> yuv_buffer_;
    std::vector<uint8_t> yuv_temp_buffer_;
    std::vector<GLubyte> rgb_buffer_;

    std::atomic<bool> is_yuv_texture_available_;
    std::atomic<bool> swap_buffer_signal_;
    std::mutex yuv_buffer_mutex_;

    size_t yuv_width_;
    size_t yuv_height_;
    size_t yuv_size_;
    size_t uv_buffer_offset_;

    bool is_service_connected_;
    bool is_texture_id_set_;
    bool is_video_overlay_rotation_set_;

    TangoSupportRotation display_rotation_;

    void AllocateTexture(GLuint texture_id, int width, int height);
    void RenderYuv();
    void RenderTextureId();
    void DeleteDrawables();

    // 用于获取用于标定的图像
    bool isCalibrationMode = false;
    cv::Mat grayImage;
    std::atomic<bool> isInCalibration;
    std::atomic<bool> shouldCalibration;
    std::mutex calibration_mutex;
    void myCalibrateCamera();
};


#endif //SENSEDATA_SENSEDATAAPP_H
