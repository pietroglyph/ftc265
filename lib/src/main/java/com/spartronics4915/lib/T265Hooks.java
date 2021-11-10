package com.spartronics4915.lib;

import android.content.Context;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;

public class T265Hooks implements OpModeManagerNotifier.Notifications {
    public static T265Camera slamera;

    private static boolean opModeIsRunning = false;

    public static boolean isOpModeRunning() {
        return opModeIsRunning;
    }

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

    /**
     * The indicated opmode is just about to be initialized.
     *
     * @param opMode The opmode that is about to be initialized.
     */
    @Override
    public void onOpModePreInit(OpMode opMode) {
        opModeIsRunning = true;
    }

    /**
     * The indicated opmode is just about to be started.
     *
     * @param opMode The opmode that is about to be started.
     */
    @Override
    public void onOpModePreStart(OpMode opMode) {
        opModeIsRunning = true;
    }

    /**
     * The indicated opmode has just been stopped.
     *
     * @param opMode The opmode that was just stopped.
     */
    @Override
    public void onOpModePostStop(OpMode opMode) {
        opModeIsRunning = false;
    }
}
