package com.spartronics4915.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseMath {
    public static final double inchesToMeters = 0.0254;
    public static final double metersToInches = 1.0 / 0.0254;

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
                t.getX() + b.getX() * cos_t - b.getY() * sin_t,
                t.getY() + b.getX() * sin_t + b.getY() * cos_t,
                t.getHeading() + b.getHeading());
    }

    /**
     * Calculates the transformation between two Pose2d objects.
     *
     * <p><a href="https://www.desmos.com/calculator/fttw74j9pp">A simulator for this behavior can
     * be found here.</a>
     *
     * @param t The origin pose.
     * @param r The result pose.
     * @return The transformation needed to get from the origin pose to the result pose.
     */
    public static Pose2d calculateTransformation(Pose2d t, Pose2d r) {
        double cos_t = Math.cos(t.getHeading());
        double sin_t = Math.sin(t.getHeading());

        return new Pose2d(
                cos_t * (r.getX() - t.getX()) + sin_t * (r.getY() - t.getY()),
                sin_t * (t.getX() - r.getX()) + cos_t * (r.getY() - t.getY()),
                r.getHeading() - t.getHeading());
    }

    /**
     * Convert a Pose2d from meters to inches.
     *
     * @param pose The pose to convert.
     * @return The converted pose.
     */
    public static Pose2d metersToInches(Pose2d pose) {
        return new Pose2d(metersToInches(pose.vec()), pose.getHeading());
    }

    /**
     * Convert a Vector2d from meters to inches.
     *
     * @param vec The vector to convert.
     * @return The converted vector.
     */
    public static Vector2d metersToInches(Vector2d vec) {
        return vec.times(metersToInches);
    }

    /**
     * Convert a Pose2d from inches to meters.
     *
     * @param pose The pose to convert.
     * @return The converted pose.
     */
    public static Pose2d inchesToMeters(Pose2d pose) {
        return new Pose2d(inchesToMeters(pose.vec()), pose.getHeading());
    }

    /**
     * Convert a Vector2d from inches to meters.
     *
     * @param vec The vector to convert.
     * @return The converted vector.
     */
    public static Vector2d inchesToMeters(Vector2d vec) {
        return vec.times(inchesToMeters);
    }
}
