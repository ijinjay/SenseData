#include <jni.h>
#include "../cpp/SenseDataApp.h"

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

    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onDisplayChanged(JNIEnv* env, jobject, jint display_rotaion) {
        app.OnDisplayChanged(display_rotaion);
    }

    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onSurfaceCreated(JNIEnv* env, jobject) {
        app.onGLSurfaceCreated();
    }
    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_onSurfaceChanged(JNIEnv* env, jobject, jint width, jint height) {
        app.onGLSurfaceChanged(width, height);
    }
    JNIEXPORT void JNICALL
    Java_io_github_jinjaysnow_sensedata_TangoJniNative_render(JNIEnv* env, jobject) {
        app.render();
    }
}
