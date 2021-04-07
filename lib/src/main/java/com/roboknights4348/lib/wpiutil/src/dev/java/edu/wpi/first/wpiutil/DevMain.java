// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.roboknights4348.lib.wpiutil.src.dev.java.edu.wpi.first.wpiutil;

import com.roboknights4348.lib.wpiutil.src.main.java.edu.wpi.first.wpiutil.RuntimeDetector;

public final class DevMain {
  /** Main entry point. */
  public static void main(String[] args) {
    System.out.println("Hello World!");
    System.out.println(RuntimeDetector.getPlatformPath());
  }

  private DevMain() {}
}
