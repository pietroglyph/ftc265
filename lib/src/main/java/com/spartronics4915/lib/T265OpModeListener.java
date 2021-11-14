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
        boolean init = true;
        String mapPath = "";
        T265Config config = opMode.getClass().getAnnotation(T265Config.class);

        if (config != null) {
            init = config.autoInit();
            mapPath = config.mapPath();
        }

        if (!init) {
            Log.i(kLogTag, "Auto init is disabled");
            return;
        }

        Log.i(kLogTag, "Auto init is enabled, initializing...");

        if (slamera == null) {
            slamera = new T265Camera(new Pose2d(), 0.8, mapPath, opMode.hardwareMap.appContext);
        }
        if (!slamera.isStarted()) {
            slamera.start();
        }
        slamera.setPose(new Pose2d());
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {}

    @Override
    public void onOpModePostStop(OpMode opMode) {}
}
