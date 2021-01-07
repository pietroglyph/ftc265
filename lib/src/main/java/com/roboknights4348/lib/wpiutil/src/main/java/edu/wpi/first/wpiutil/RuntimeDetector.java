// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.roboknights4348.lib.wpiutil.src.main.java.edu.wpi.first.wpiutil;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public final class RuntimeDetector {
  private static String filePrefix;
  private static String fileExtension;
  private static String filePath;

  @SuppressWarnings("PMD.CyclomaticComplexity")
  private static synchronized void computePlatform() {
    if (fileExtension != null && filePath != null && filePrefix != null) {
      return;
    }

    boolean intel32 = is32BitIntel();
    boolean intel64 = is64BitIntel();

    if (isWindows()) {
      filePrefix = "";
      fileExtension = ".dll";
      if (intel32) {
        filePath = "/windows/x86/";
      } else {
        filePath = "/windows/x86-64/";
      }
    } else if (isMac()) {
      filePrefix = "lib";
      fileExtension = ".dylib";
      if (intel32) {
        filePath = "/osx/x86";
      } else {
        filePath = "/osx/x86-64/";
      }
    } else if (isLinux()) {
      filePrefix = "lib";
      fileExtension = ".so";
      if (intel32) {
        filePath = "/linux/x86/";
      } else if (intel64) {
        filePath = "/linux/x86-64/";
      } else if (isAthena()) {
        filePath = "/linux/athena/";
      } else if (isRaspbian()) {
        filePath = "/linux/raspbian/";
      } else if (isAarch64()) {
        filePath = "/linux/aarch64bionic/";
      } else {
        filePath = "/linux/nativearm/";
      }
    } else {
      throw new IllegalStateException("Failed to determine OS");
    }
  }

  /** Get the file prefix for the current system. */
  public static synchronized String getFilePrefix() {
    computePlatform();

    return filePrefix;
  }

  /** Get the file extension for the current system. */
  public static synchronized String getFileExtension() {
    computePlatform();

    return fileExtension;
  }

  /** Get the platform path for the current system. */
  public static synchronized String getPlatformPath() {
    computePlatform();

    return filePath;
  }

  /** Get the path to the requested resource. */
  public static synchronized String getLibraryResource(String libName) {
    computePlatform();

    return filePath + filePrefix + libName + fileExtension;
  }

  /** Get the path to the hash to the requested resource. */
  public static synchronized String getHashLibraryResource(String libName) {
    computePlatform();

    return filePath + libName + ".hash";
  }

  public static boolean isAthena() {
    File runRobotFile = new File("/usr/local/frc/bin/frcRunRobot.sh");
    return runRobotFile.exists();
  }

  /**
   * check if os is raspbian.
   *
   * @return if os is raspbian
   */
  public static boolean isRaspbian() {
    String value = null;
    if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
      try (BufferedReader reader = Files.newBufferedReader(Paths.get("/etc/os-release"))) {
        value = reader.readLine();
      } catch (IOException ex) {
        return false;
      }
    }
    return value.contains("Raspbian");
  }

  /**
   * check if architecture is aarch64.
   *
   * @return if architecture is aarch64
   */
  public static boolean isAarch64() {
    return System.getProperty("os.arch").equals("aarch64");
  }

  public static boolean isLinux() {
    return System.getProperty("os.name").startsWith("Linux");
  }

  public static boolean isWindows() {
    return System.getProperty("os.name").startsWith("Windows");
  }

  public static boolean isMac() {
    return System.getProperty("os.name").startsWith("Mac");
  }

  public static boolean is32BitIntel() {
    String arch = System.getProperty("os.arch");
    return "x86".equals(arch) || "i386".equals(arch);
  }

  public static boolean is64BitIntel() {
    String arch = System.getProperty("os.arch");
    return "amd64".equals(arch) || "x86_64".equals(arch);
  }

  private RuntimeDetector() {}
}
