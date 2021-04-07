// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.kinematics;

import java.util.stream.DoubleStream;

@SuppressWarnings("MemberName")
public class MecanumDriveWheelSpeeds {
  /** Speed of the front left wheel. */
  public double frontLeftMetersPerSecond;

  /** Speed of the front right wheel. */
  public double frontRightMetersPerSecond;

  /** Speed of the rear left wheel. */
  public double rearLeftMetersPerSecond;

  /** Speed of the rear right wheel. */
  public double rearRightMetersPerSecond;

  /** Constructs a MecanumDriveWheelSpeeds with zeros for all member fields. */
  public MecanumDriveWheelSpeeds() {}

  /**
   * Constructs a MecanumDriveWheelSpeeds.
   *
   * @param frontLeftMetersPerSecond Speed of the front left wheel.
   * @param frontRightMetersPerSecond Speed of the front right wheel.
   * @param rearLeftMetersPerSecond Speed of the rear left wheel.
   * @param rearRightMetersPerSecond Speed of the rear right wheel.
   */
  public MecanumDriveWheelSpeeds(
      double frontLeftMetersPerSecond,
      double frontRightMetersPerSecond,
      double rearLeftMetersPerSecond,
      double rearRightMetersPerSecond) {
    this.frontLeftMetersPerSecond = frontLeftMetersPerSecond;
    this.frontRightMetersPerSecond = frontRightMetersPerSecond;
    this.rearLeftMetersPerSecond = rearLeftMetersPerSecond;
    this.rearRightMetersPerSecond = rearRightMetersPerSecond;
  }

  /**
   * Normalizes the wheel speeds using some max attainable speed. Sometimes, after inverse
   * kinematics, the requested speed from a/several modules may be above the max attainable speed
   * for the driving motor on that module. To fix this issue, one can "normalize" all the wheel
   * speeds to make sure that all requested module speeds are below the absolute threshold, while
   * maintaining the ratio of speeds between modules.
   *
   * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a wheel can reach.
   */
  public void normalize(double attainableMaxSpeedMetersPerSecond) {
    double realMaxSpeed =
        DoubleStream.of(
                frontLeftMetersPerSecond,
                frontRightMetersPerSecond,
                rearLeftMetersPerSecond,
                rearRightMetersPerSecond)
            .max()
            .getAsDouble();

    if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
      frontLeftMetersPerSecond =
          frontLeftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      frontRightMetersPerSecond =
          frontRightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      rearLeftMetersPerSecond =
          rearLeftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      rearRightMetersPerSecond =
          rearRightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
    }
  }

  @Override
  public String toString() {
    return String.format(
        "MecanumDriveWheelSpeeds(Front Left: %.2f m/s, Front Right: %.2f m/s, "
            + "Rear Left: %.2f m/s, Rear Right: %.2f m/s)",
        frontLeftMetersPerSecond,
        frontRightMetersPerSecond,
        rearLeftMetersPerSecond,
        rearRightMetersPerSecond);
  }
}
