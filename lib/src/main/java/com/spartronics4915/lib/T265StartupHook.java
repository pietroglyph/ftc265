package com.spartronics4915.lib;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;

public class T265StartupHook {
    public static T265Camera slamera;

    @OnCreate
    public static void createSlamera(Context appContext) {
        slamera = new T265Camera(new Pose2d(), 0.8, appContext);
        slamera.start();
    }

    @OnDestroy
    public static void destroySlamera(Context appContext) {
        slamera.stop();
        slamera.free();
    }
}
