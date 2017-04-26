package io.github.jinjaysnow.sensedata;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.opengl.GLSurfaceView;
import android.os.IBinder;
import android.provider.Settings;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Window;
import android.view.WindowManager;
import android.widget.TextView;
import android.content.ServiceConnection;


public class MainActivity extends AppCompatActivity {

    // 加载静态库文件
    static {
        System.loadLibrary("sense-data-app");
    }

    private GLSurfaceView mGLView;
    // tango服务绑定
    public static final boolean bindTangoService(final Context context, ServiceConnection connection) {
        Intent intent = new Intent();
        intent.setClassName("com.google.tango", "com.google.atap.tango.TangoService");

        boolean hasJavaService = (context.getPackageManager().resolveService(intent, 0) != null);
        // 查找前一版本的Tango Core服务
        if (!hasJavaService) {
            intent = new Intent();
            intent.setClassName("com.projecttango.tango", "com.google.atap.tango.TangoService");
            hasJavaService = (context.getPackageManager().resolveService(intent, 0) != null);
        }
        // 缺乏Tango服务
        if (!hasJavaService) {
            return false;
        }

        return context.bindService(intent, connection, Context.BIND_AUTO_CREATE);
    }

    // 连接Tango服务
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            synchronized (MainActivity.this) {
                TangoJniNative.onTangoServiceConnected(service);
            }
        }
        public void onServiceDisconnected(ComponentName name) {

        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 设置全屏
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);

//        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

        // 初始化Tango和IMU传感器
        TangoJniNative.onCreate(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        // 绑定Tango服务
        bindTangoService(this, mTangoServiceConnection);
        // IMU数据
        TangoJniNative.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
        TangoJniNative.onPause();
        unbindService(mTangoServiceConnection);
    }

    @Override
    protected void onStop(){
        super.onStop();
    }

}
