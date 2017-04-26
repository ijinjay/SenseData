#include <jni.h>
#include "../cpp/SenseDataApp.h"
#include <android/log.h>

#define  LOG_TAG    "SenseDataApp"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static SenseDataApp app;

extern "C" {
    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onCreate(JNIEnv *env, jobject /* this */, jobject activity) {
        LOGI("enter oncreate");
        app.onCreate(env, activity);
    }

    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onTangoServiceConnected(JNIEnv* env, jobject, jobject binder) {
        app.onTangoServiceConnected(env,binder);
    }
    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onPause(JNIEnv* env, jobject) {
        app.onPause();
    }
    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onResume(JNIEnv* env, jobject) {
        app.onResume();
    }
}
