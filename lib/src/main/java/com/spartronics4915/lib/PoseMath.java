package com.spartronics4915.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseMath {
    /**
     * Transforms a Pose2d by another Pose2d.
     *
     * @param t The first pose.
     * @param b The pose to transform the first pose by.
     * @return The result of the transformation.
     */
    public static Pose2d transformBy(Pose2d t, Pose2d b) {
        double cos_t = Math.cos(t.getHeading());
        double sin_t = Math.sin(t.getHeading());

        double cos_b = Math.cos(b.getHeading());
        double sin_b = Math.sin(b.getHeading());

        return new Pose2d(
                t.getX() + (b.getX() * cos_t - b.getY() * sin_t),
                t.getY() + (b.getX() * sin_t - b.getY() * cos_t),
                Math.atan2(cos_t * sin_b + sin_t * cos_b, cos_t * cos_b - sin_t * sin_b));
    }

    /**
     * Returns the other pose relative to the current pose.
     *
     * @param initial The initial pose.
     * @param other The pose that is the origin of the new coordinate frame that the initial pose
     *     will be converted into.
     * @return The initial pose relative to the new origin pose.
     */
    public static Pose2d relativeTo(Pose2d initial, Pose2d other) {
        double other_sin = Math.sin(-other.getHeading());
        double other_cos = Math.cos(-other.getHeading());

        double x_diff = other.getX() - initial.getX();
        double y_diff = other.getY() - initial.getY();

        return new Pose2d(
                x_diff * other_cos - y_diff * other_sin,
                y_diff * other_sin + y_diff * other_cos,
                other.getHeading() - initial.getHeading());
    }
}
