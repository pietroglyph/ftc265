package com.spartronics4915.lib;

import static com.spartronics4915.lib.T265Hooks.slamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** Localizer for the T265 camera. */
@SuppressWarnings("unused")
public class T265Localizer implements Localizer, Runnable {
    // Lock so we can only have one update at a time
    private final Object updateMutex = new Object();
    /**
     * This will update the T265's odometry, among other things, separately from the main thread.
     */
    @Override
    public void run() {
        // We only want this to run while the op mode is active so the GC can do its thing.
        while (T265Hooks.isOpModeRunning()) {
            update();
        }
    }

    // Interface so we can pass a callback as a parameter
    public interface SendOdometryFunction {
        Vector2d run();
    }
    // This is the callback that gets called when we update the localizer.
    private SendOdometryFunction sendOdometryCallback;

    // Caches the last update we got from the T265
    private T265Camera.CameraUpdate lastReceivedCameraUpdate;

    /**
     * Get the current pose of the T265.
     *
     * @return the pose
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return lastReceivedCameraUpdate.pose;
    }

    /**
     * Set the pose of the T265 camera.
     *
     * @param pose2d the pose
     */
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        slamera.setPose(pose2d);
    }

    /**
     * Get the current velocity of the T265.
     *
     * @return the velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return lastReceivedCameraUpdate.velocity;
    }

    /**
     * Get the current pose confidence of the T265. You should call this right after calling
     * getPositionEstimate() to get the most up-to-date pose confidence.
     *
     * @return the confidence
     */
    public T265Camera.PoseConfidence getPoseConfidence() {
        return lastReceivedCameraUpdate.confidence;
    }

    /** Send odometry data to the T265 if the callback is set. */
    @Override
    public synchronized void update() {
        synchronized (updateMutex) {
            // Make sure the callback is set
            if (sendOdometryCallback != null) {
                Vector2d odometry = sendOdometryCallback.run();
                slamera.sendOdometry(odometry.getX(), odometry.getY());
            }

            // Get the latest update from the T265
            lastReceivedCameraUpdate = slamera.getLastReceivedCameraUpdate();
        }
    }

    /**
     * Set a callback to send odometry data to the T265.
     *
     * @param sendOdometryCallback the callback
     */
    public void setSendOdometryCallback(SendOdometryFunction sendOdometryCallback) {
        this.sendOdometryCallback = sendOdometryCallback;
    }

    /**
     * Create a new T265Localizer. We actually don't need the hardware map, but it's here for
     * consistency with other localizers
     *
     * @param hardwareMap the hardware map from the op mode
     */
    public T265Localizer(HardwareMap hardwareMap) {
        this();
    }

    /** Create a new T265Localizer. */
    public T265Localizer() {
        Thread updateThread = new Thread(this, "T265 Update");
        updateThread.start();
    }
}
