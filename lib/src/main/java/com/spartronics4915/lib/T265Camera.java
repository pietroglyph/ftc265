package com.spartronics4915.lib;

import android.content.Context;
import android.util.Log;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.UsbUtilities;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

/**
 * Provides a convenient Java interface to the Intel RealSense T265 V-SLAM camera. Only the subset
 * of the librealsense that is useful to robot tracking is exposed in this class.
 *
 * <p>We employ JNI to call librealsense. There <i>are</i> Java bindings for librealsense, but they
 * are not complete and do not support our usecase.
 *
 * <p>This class works entirely in 2d, even though the tracking camera supports giving us a third
 * dimension (Z).
 *
 * <p>The coordinate system is as follows: + X == Robot forwards + Y == Robot left (left is from the
 * perspective of a viewer standing behind the robot)
 *
 * <p>All distance units are meters. All time units are seconds.
 */
public class T265Camera {
    private static final String kLogTag = "ftc265";

    private static UnsatisfiedLinkError mLinkError = null;

    static {
        try {
            Log.d(kLogTag, "Attempting to load native code");

            System.loadLibrary("ftc265");

            // Cleanup is quite tricky for us, because the native code has no idea when Java
            // will be done. (This is why smart pointers don't really make sense in the native
            // code.)
            // Even worse, trying to cleanup with atexit in the native code is too late and
            // results in unfinished callbacks blocking. As a result a shutdown hook is our
            // best option.
            Runtime.getRuntime().addShutdownHook(new Thread(T265Camera::cleanup));
        } catch (UnsatisfiedLinkError e) {
            Log.e(kLogTag, "Failed to load native code", e);
            mLinkError = e;
        }
    }

    public static enum PoseConfidence {
        Failed,
        Low,
        Medium,
        High,
    }

    public static class CameraUpdate {
        /** The robot's pose in meters. */
        public final Pose2d pose;
        /** The robot's velocity in meters/sec and radians/sec. */
        public final ChassisSpeeds velocity;

        public final PoseConfidence confidence;

        public CameraUpdate(Pose2d pose, ChassisSpeeds velocity, PoseConfidence confidence) {
            this.pose = pose;
            this.velocity = velocity;
            this.confidence = confidence;
        }
    }

    private final AtomicBoolean mInitInProgress = new AtomicBoolean(false);

    // Protected by mPointerMutex
    private final Object mPointerMutex = new Object();
    private boolean mHasSeenDeviceBefore = false;
    private long mNativeCameraObjectPointer = 0;

    // Protected by a mutex on this
    private boolean mIsStarted = false;
    private Transform2d mRobotOffset;
    private Pose2d mOriginOffset = new Pose2d();

    // Protected by mUpdateMutex
    private final Object mUpdateMutex = new Object();
    private CameraUpdate mLastRecievedUpdate = null;
    private Consumer<CameraUpdate> mPoseConsumer = null;

    /**
     * This method constructs a T265 camera and sets it up with the right info. {@link
     * T265Camera#start start} will not be called, you must call it yourself.
     *
     * @param robotOffset Offset of the center of the robot from the center of the camera.
     * @param odometryCovariance Covariance of the odometry input when doing sensor fusion (you
     *     probably want to tune this).
     */
    public T265Camera(Transform2d robotOffset, double odometryCovariance, Context appContext) {
        this(robotOffset, odometryCovariance, "", appContext);
    }

    /**
     * This method constructs a T265 camera and sets it up with the right info. {@link
     * T265Camera#start start} will not be called, you must call it yourself.
     *
     * @param robotOffsetMeters Offset of the center of the robot from the center of the camera.
     *     Units are meters.
     * @param odometryCovariance Covariance of the odometry input when doing sensor fusion (you
     *     probably want to tune this)
     * @param relocMapPath path (including filename) to a relocalization map to load.
     */
    public T265Camera(
            Transform2d robotOffsetMeters,
            double odometryCovariance,
            String relocMapPath,
            Context appContext) {
        if (mLinkError != null) {
            throw mLinkError;
        }

        DeviceListener callback =
                new DeviceListener() {
                    @Override
                    public void onDeviceAttach() {
                        try {
                            mInitInProgress.set(true);

                            // This check assumes that there's only one camera
                            synchronized (mPointerMutex) {
                                if (mNativeCameraObjectPointer != 0) {
                                    return;
                                }
                            }

                            Log.i(
                                    kLogTag,
                                    "onDeviceAttached called... Will attempt to handoff to native code.");

                            long ptr = newCamera(relocMapPath);
                            synchronized (mPointerMutex) {
                                // newCamera is hoisted out to before we lock because it can block
                                // for a while
                                mHasSeenDeviceBefore |= ptr != 0;
                                mNativeCameraObjectPointer = ptr;
                            }
                            setOdometryInfo(
                                    (float) robotOffsetMeters.getTranslation().getX(),
                                    (float) robotOffsetMeters.getTranslation().getY(),
                                    (float) robotOffsetMeters.getRotation().getRadians(),
                                    odometryCovariance);
                            mRobotOffset = robotOffsetMeters;

                            Log.d(kLogTag, "Native code should be done initializing");
                        } catch (Exception e) {
                            Log.e(
                                    kLogTag,
                                    "Exception while initializing camera (could be spurious if camera was initially presented as a generic Movidius device as part of the init process)",
                                    e);
                        } finally {
                            Log.d(kLogTag, "Setting initInProgress to false");
                            mInitInProgress.set(false);
                        }
                    }

                    @Override
                    public void onDeviceDetach() {
                        Log.d(kLogTag, "onDeviceDetach called...");
                        synchronized (mPointerMutex) {
                            // Unfortunately we don't get any information about the detaching
                            // device, which means that any rs device detaching will detach *all*
                            // other devices. This is one of the few blockers for multi-device
                            // support.
                            if (mNativeCameraObjectPointer != 0) {
                                Log.i(
                                        kLogTag,
                                        "onDeviceDetach called with non-null native pointer. Will attempt to free native objects.");
                                free();
                            }
                        }
                    }

                    @Override
                    public boolean equals(Object obj) {
                        return false;
                    }
                };

        Log.d(kLogTag, "Initializing RsContext and asking for permissions...");
        RsContext.init(appContext, callback);

        synchronized (mPointerMutex) {
            int numDevices = UsbUtilities.getDevices(appContext).size();
            Log.d(kLogTag, "Found " + numDevices + " devices at init");
            mHasSeenDeviceBefore = numDevices > 0;
        }
    }

    /**
     * This allows the {@link T265Camera#getLastReceivedCameraUpdate()} to start returning pose
     * data. This will NOT reset the camera's pose.
     *
     * <p>This will not restart the camera following {@link
     * T265Camera#exportRelocalizationMap(String)}. You will have to call {@link T265Camera#free()}
     * and make a new {@link T265Camera}. This is related to what appears to be a bug in
     * librealsense.
     *
     * @throws RuntimeException This will throw if the camera isn't connected or the camera has
     *     already been started.
     */
    public void start() {
        start(
                (update) -> {
                    synchronized (mUpdateMutex) {
                        mLastRecievedUpdate = update;
                    }
                });
    }

    /**
     * This allows the user-provided pose receive callback to receive data. This will NOT reset the
     * camera's pose. This is the advanced version of the start method; if you don't want to provide
     * a callback and just want to call {@link T265Camera#getLastReceivedCameraUpdate()} instead
     * then you should call {@link T265Camera#start()}.
     *
     * <p>This will not restart the camera following {@link
     * T265Camera#exportRelocalizationMap(String)}. You will have to call {@link T265Camera#free()}
     * and make a new {@link T265Camera}. This is related to what appears to be a bug in
     * librealsense.
     *
     * @param poseConsumer A method to be called every time we receive a pose from <i>from a
     *     different thread</i>! You must synchronize memory access across threads!
     *     <p>Received poses are in meters.
     * @throws RuntimeException This will throw if the camera isn't connected. This will never throw
     *     following one successful connection; we will instead continue to try and reconnect.
     */
    public synchronized void start(Consumer<CameraUpdate> poseConsumer) {
        Log.d(kLogTag, "Trying to start camera callback");

        synchronized (mPointerMutex) {
            if (mIsStarted) throw new RuntimeException("Camera is already started");
            else if (mNativeCameraObjectPointer == 0
                    && !mHasSeenDeviceBefore
                    && !mInitInProgress.get()) {
                throw new RuntimeException("No camera connected");
            }
        }

        synchronized (mUpdateMutex) {
            mLastRecievedUpdate = null;
        }

        mPoseConsumer = poseConsumer;
        mIsStarted = true;

        Log.d(kLogTag, "Camera callback should be started");
    }

    /**
     * @return Whether or not the camera is started. Note: the camera driver can still be
     *     initializing while it is started.
     */
    public synchronized boolean isStarted() {
        return mIsStarted;
    }

    /**
     * Blocks until a new camera update comes in. The camera update will include the latest pose
     * estimate.
     *
     * @return The last received camera update, or null if a custom callback was passed to {@link
     *     T265Camera#start(Consumer)}.
     */
    public CameraUpdate getLastReceivedCameraUpdate() {
        synchronized (mUpdateMutex) {
            if (mLastRecievedUpdate == null) {
                Log.w(
                        kLogTag,
                        "Attempt to get last received update before any updates have been received; are you using the wrong T265Camera::start overload, or is the camera not initialized yet or busy?");
                return new CameraUpdate(new Pose2d(), new ChassisSpeeds(), PoseConfidence.Failed);
            }
            return mLastRecievedUpdate;
        }
    }

    /** This stops the callback from receiving data, but it does not internally stop the camera. */
    public synchronized void stop() {
        Log.d(kLogTag, "Stopping camera callback");

        mIsStarted = false;
    }

    /**
     * Exports a binary relocalization map file to the given path. This will stop the camera.
     * Because of a librealsense bug the camera isn't restarted after you call this method. TODO:
     * Fix that.
     *
     * @param path Path (with filename) to export to
     */
    public native void exportRelocalizationMap(String path);

    /**
     * Sends robot velocity as computed from wheel encoders. Note that the X and Y axis orientations
     * are determined by how you set the robotOffset in the constructor.
     *
     * @param velocityXMetersPerSecond The robot-relative velocity along the X axis in meters/sec.
     * @param velocityYMetersPerSecond The robot-relative velocity along the Y axis in meters/sec.
     */
    public void sendOdometry(double velocityXMetersPerSecond, double velocityYMetersPerSecond) {
        synchronized (mPointerMutex) {
            if (mNativeCameraObjectPointer == 0)
                Log.w(kLogTag, "Can't send odometry while camera is busy or not initialized yet");
        }

        // Only 1 odometry sensor is supported for now (index 0)
        sendOdometryRaw(0, (float) velocityXMetersPerSecond, (float) velocityYMetersPerSecond);
    }

    /**
     * This zeroes the camera pose to the provided new pose.
     *
     * @param newPose The pose the camera should be zeroed to.
     */
    public synchronized void setPose(Pose2d newPose) {
        synchronized (mUpdateMutex) {
            mOriginOffset =
                    newPose.relativeTo(
                            mLastRecievedUpdate == null ? new Pose2d() : mLastRecievedUpdate.pose);
        }
    }

    /**
     * This will free the underlying native objects. You probably don't want to use this; on program
     * shutdown the native code will gracefully stop and delete any remaining objects.
     */
    public native void free();

    private native void setOdometryInfo(
            float robotOffsetX,
            float robotOffsetY,
            float robotOffsetRads,
            double measurementCovariance);

    private native void sendOdometryRaw(int sensorIndex, float xVel, float yVel);

    private native long newCamera(String mapPath);

    private static native void cleanup();

    private synchronized void consumePoseUpdate(
            float x, float y, float radians, float dx, float dy, float dtheta, int confOrdinal) {
        // First we apply an offset to go from the camera coordinate system to the
        // robot coordinate system with an origin at the center of the robot. This
        // is not a directional transformation.
        // Then we transform the pose our camera is giving us so that it reports is
        // the robot's pose, not the camera's. This is a directional transformation.
        final Pose2d currentPose =
                new Pose2d(
                                x - mRobotOffset.getTranslation().getX(),
                                y - mRobotOffset.getTranslation().getY(),
                                new Rotation2d(radians))
                        .transformBy(mRobotOffset);

        if (!mIsStarted) return;

        // See
        // https://github.com/IntelRealSense/librealsense/blob/7f2ba0de8769620fd672f7b44101f0758e7e2fb3/include/librealsense2/h/rs_types.h#L115
        // for ordinals
        PoseConfidence confidence;
        switch (confOrdinal) {
            case 0x0:
                confidence = PoseConfidence.Failed;
                break;
            case 0x1:
                confidence = PoseConfidence.Low;
                break;
            case 0x2:
                confidence = PoseConfidence.Medium;
                break;
            case 0x3:
                confidence = PoseConfidence.High;
                break;
            default:
                throw new RuntimeException(
                        "Unknown confidence ordinal \""
                                + confOrdinal
                                + "\" passed from native code");
        }

        final Pose2d transformedPose =
                mOriginOffset.transformBy(
                        new Transform2d(currentPose.getTranslation(), currentPose.getRotation()));

        mPoseConsumer.accept(
                new CameraUpdate(transformedPose, new ChassisSpeeds(dx, dy, dtheta), confidence));
    }

    /** Thrown if something goes wrong in the native code */
    public static class CameraJNIException extends RuntimeException {

        // This must be static _and_ have this constructor if you want it to be
        // thrown from native code
        public CameraJNIException(String message) {
            super(message);
        }
    }
}
