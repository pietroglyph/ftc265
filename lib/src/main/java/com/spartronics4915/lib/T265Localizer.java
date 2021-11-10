package com.spartronics4915.lib;

import static com.spartronics4915.lib.T265StartupHook.slamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** Localizer for the T265 camera. */
public class T265Localizer implements Localizer {
    public interface SendOdometryFunction {
        Vector2d run();
    }

    private SendOdometryFunction sendOdometryCallback;

    /**
     * Get the current pose of the T265.
     *
     * @return the pose
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return slamera.getLastReceivedCameraUpdate().pose;
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
        return slamera.getLastReceivedCameraUpdate().velocity;
    }

    /** Send odometry data to the T265 if the callback is set. */
    @Override
    public void update() {
        // Make sure the callback is set
        if (sendOdometryCallback != null) {
            Vector2d odometry = sendOdometryCallback.run();
            slamera.sendOdometry(odometry.getX(), odometry.getY());
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
}
