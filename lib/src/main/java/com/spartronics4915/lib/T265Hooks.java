package com.spartronics4915.lib;

import android.content.Context;
import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public class T265Hooks implements OpModeManagerNotifier.Notifications {
    private static final String kLogTag = "ftc265";
    public static T265Camera slamera;

    /**
     * Add a listener to initialize the T265Camera when an opmode is created.
     *
     * @param appContext Passed in by the FTC SDK
     * @param eventLoop Passed in by the FTC SDK
     */
    @OnCreateEventLoop
    public static void onCreateEventLoop(Context appContext, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(new T265Hooks());
    }

    /**
     * Initialize the T265Camera when the indicated opmode is created if there is no T265Camera yet.
     *
     * @param opMode The opmode that is about to be initialized
     */
    @Override
    public void onOpModePreInit(OpMode opMode) {
        Log.d(kLogTag, "Creating T265Camera");
        if (slamera == null) {
            slamera = new T265Camera(new Pose2d(), 0.8, opMode.hardwareMap.appContext);
        }
        if (!slamera.isStarted()) {
            slamera.start();
        }
        Log.d(kLogTag, "T265Camera created");
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {}

    @Override
    public void onOpModePostStop(OpMode opMode) {}
}
