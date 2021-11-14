package com.spartronics4915.lib;

import static com.spartronics4915.lib.T265AutoInit.slamera;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

public class T265OpModeListener implements OpModeManagerNotifier.Notifications {
    private static final String kLogTag = "ftc265";

    /**
     * Initialize the T265Camera when the indicated opmode is created if there is no T265Camera yet.
     *
     * @param opMode The opmode that is about to be initialized
     */
    @Override
    public void onOpModePreInit(OpMode opMode) {
        Log.d(kLogTag, "Checking if the T265 needs to be initialized...");
        if (slamera == null) {
            Log.i(kLogTag, "Initializing T265AutoInit.slamera");
            slamera = new T265Camera(new Pose2d(), 0.8, opMode.hardwareMap.appContext);
        }
        Log.d(kLogTag, "Checking if the T265 needs to be started...");
        if (!slamera.isStarted()) {
            Log.i(kLogTag, "Starting T265AutoInit.slamera");
            slamera.start();
        }
        Log.i(kLogTag, "T265AutoInit.slamera should be ready to use!");
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {}

    @Override
    public void onOpModePostStop(OpMode opMode) {}
}
