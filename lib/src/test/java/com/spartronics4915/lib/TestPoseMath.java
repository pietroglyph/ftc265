package com.spartronics4915.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.junit.Test;

import java.util.ArrayList;

import static junit.framework.TestCase.fail;

public class TestPoseMath {
    @Test
    public void testTransform() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            poses.add(new Pose2d(i, i, i));
        }

        for (Pose2d pose : poses) {
            for (Pose2d transform : poses) {
                Pose2d result = PoseMath.transformBy(pose, transform);
                Pose2d calculatedTransform = PoseMath.calculateTransformation(pose, result);
                if (!transform.epsilonEquals(calculatedTransform)) {
                    fail("Transformations failed:" +
                            "\nPose: " + pose +
                            "\nTransform: " + transform +
                            "\nResult: " + result +
                            "\nCalculatedTransform: " + calculatedTransform);
                }
            }
        }
    }

    @Test
    public void testConversions() {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (int i = 0; i < 100; i++) {
            poses.add(new Pose2d(i, i, i));
        }

        for (Pose2d pose : poses) {
            Pose2d inMeters = PoseMath.inchesToMeters(pose);
            Pose2d inInches = PoseMath.metersToInches(inMeters);

            // We can't use epsilonEquals because of the floating point errors
            if (pose.getX() - inInches.getX() +
                    pose.getY() - inInches.getY() +
                    pose.getHeading() - inInches.getHeading()
                    > 0.0001) {
                fail("Conversions failed:" +
                        "\nPose: " + pose +
                        "\nInMeters: " + inMeters +
                        "\nInInches: " + inInches);
            }
        }
    }
}
