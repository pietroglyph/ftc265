package com.spartronics4915.lib;

import android.content.Context;
import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.FtcEventLoop;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;

public class T265Hooks {
    private static final String kLogTag = "ftc265";
    public static T265Camera slamera;

    @OnCreateEventLoop
    public static void attachEventLoop(Context appContext, FtcEventLoop eventLoop) {
        Log.d(kLogTag, "Creating T265Camera");
        slamera = new T265Camera(new Pose2d(), 0.8, appContext);
        slamera.start();
        Log.d(kLogTag, "T265Camera created");
    }

    @OnDestroy
    public static void destroySlamera(Context appContext) {
        Log.d(kLogTag, "Destroying T265Camera");
        slamera.stop();
        slamera.free();
        Log.d(kLogTag, "T265Camera destroyed");
    }
}
