package io.github.jinjaysnow.sensedata;

import android.app.Activity;
import android.os.IBinder;
import android.util.Log;

/**
 * Created by jinjay on 2017/4/17.
 */

public class TangoJniNative {

    public static native void onCreate(Activity callerActivity);

    public static native void onTangoServiceConnected(IBinder binder);
    public static native void onResume();

    public static native void onPause();

}
