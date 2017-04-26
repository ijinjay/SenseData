//
// Created by JinJay on 2017/4/17.
//

#ifndef SENSEDATA_SENSEDATAAPP_H
#define SENSEDATA_SENSEDATAAPP_H

#include <jni.h>


/// 应用程序类
class SenseDataApp {
public:
    SenseDataApp();
    ~SenseDataApp();
    void onCreate(JNIEnv* env, jobject activity);
    void onTangoServiceConnected(JNIEnv* env, jobject activity);
    void onResume();
    void onPause();
};


#endif //SENSEDATA_SENSEDATAAPP_H
