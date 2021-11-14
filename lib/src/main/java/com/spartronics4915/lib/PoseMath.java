package com.spartronics4915.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseMath {
    /**
     * Transforms a Pose2d by another Pose2d.
     *
     * <p><a href="https://www.desmos.com/calculator/0x7xoeiudh">A simulator for this behavior can
     * be found here.</a>
     *
     * @param t The first pose.
     * @param b The pose to transform the first pose by.
     * @return The result of the transformation.
     */
    public static Pose2d transformBy(Pose2d t, Pose2d b) {
        double cos_t = Math.cos(t.getHeading());
        double sin_t = Math.sin(t.getHeading());

        return new Pose2d(
                t.getX() + (b.getX() * cos_t - b.getY() * sin_t),
                t.getY() + (b.getX() * sin_t + b.getY() * cos_t),
                t.getHeading() + b.getHeading());
    }

    /**
     * Returns the other pose relative to the current pose.
     *
     * @param t The initial pose.
     * @param r The pose that is the origin of the new coordinate frame that the initial pose will
     *     be converted into.
     * @return The initial pose relative to the new origin pose.
     */
    public static Pose2d relativeTo(Pose2d t, Pose2d r) {
        double cos_t = Math.cos(t.getHeading());
        double sin_t = Math.sin(t.getHeading());
        double tan_t = Math.tan(t.getHeading());

        return new Pose2d(
                (r.getX() - t.getX())
                        + (r.getY() - t.getY()) * (sin_t / cos_t) / (1 + tan_t * tan_t) / cos_t,
                (r.getY() - t.getY())
                        - (r.getX() + t.getX()) * (sin_t / cos_t) / (1 + tan_t * tan_t) / cos_t,
                r.getHeading() - t.getHeading());
    }
}
