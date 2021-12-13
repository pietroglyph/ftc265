package com.spartronics4915.lib;

import android.content.Context;

/**
 * This class makes it easy to use the T265 camera. Once you call {@link
 * #getCamera(T265Camera.OdometryInfo, Context)}, it generates a new camera object that is preserved
 * between op modes. To destroy the camera, call {@link #destroyCamera()}.
 */
@SuppressWarnings("unused")
public class T265Helper {
    private static T265Camera slamera;

    /**
     * Get the camera. Creates a new camera if one does not exist.
     *
     * @param odometryInfo the odometry info to use.
     * @param appContext the application context, usually from hardwareMap.appContext
     * @see T265Camera#T265Camera(T265Camera.OdometryInfo, Context)
     * @return the camera
     */
    public static T265Camera getCamera(T265Camera.OdometryInfo odometryInfo, Context appContext) {
        return getCamera(odometryInfo, "", appContext);
    }

    /**
     * Get the camera. Creates a new camera if one does not exist.
     *
     * @param odometryInfo the odometry info to use.
     * @param relocMapPath path (including filename) to a relocalization map to load.
     * @param appContext the application context, usually from hardwareMap.appContext
     * @see T265Camera#T265Camera(T265Camera.OdometryInfo, String, Context)
     * @return the camera
     */
    public static T265Camera getCamera(
            T265Camera.OdometryInfo odometryInfo, String relocMapPath, Context appContext) {
        if (slamera == null) {
            slamera = new T265Camera(odometryInfo, relocMapPath, appContext);
        }
        return slamera;
    }

    /**
     * Destroy the camera. You probably don't want to use this.
     *
     * @see T265Camera#stop()
     * @see T265Camera#free()
     */
    public static void destroyCamera() {
        if (slamera != null) {
            if (slamera.isStarted()) {
                slamera.stop();
            }
            slamera.free();
            slamera = null;
        }
    }
}
