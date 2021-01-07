// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.math;

import com.roboknights4348.lib.wpiutil.src.main.java.edu.wpi.first.wpiutil.RuntimeLoader;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;



public final class WPIMathJNI {
  static boolean libraryLoaded = false;
  static RuntimeLoader<WPIMathJNI> loader = null;

  static {
    if (Helper.getExtractOnStaticLoad()) {
      try {
        loader =
            new RuntimeLoader<>(
                "wpimathjni", RuntimeLoader.getDefaultExtractionRoot(), WPIMathJNI.class);
        loader.loadLibrary();
      } catch (IOException ex) {
        ex.printStackTrace();
        System.exit(1);
      }
      libraryLoaded = true;
    }
  }

  /**
   * Force load the library.
   *
   * @throws IOException If the library could not be loaded or found.
   */
  public static synchronized void forceLoad() throws IOException {
    if (libraryLoaded) {
      return;
    }
    loader =
        new RuntimeLoader<>(
            "wpimathjni", RuntimeLoader.getDefaultExtractionRoot(), WPIMathJNI.class);
    loader.loadLibrary();
    libraryLoaded = true;
  }

  /**
   * Solves the discrete alegebraic Riccati equation.
   *
   * @param A Array containing elements of A in row-major order.
   * @param B Array containing elements of B in row-major order.
   * @param Q Array containing elements of Q in row-major order.
   * @param R Array containing elements of R in row-major order.
   * @param states Number of states in A matrix.
   * @param inputs Number of inputs in B matrix.
   * @param S Array storage for DARE solution.
   */
  public static native void discreteAlgebraicRiccatiEquation(
      double[] A, double[] B, double[] Q, double[] R, int states, int inputs, double[] S);

  /**
   * Computes the matrix exp.
   *
   * @param src Array of elements of the matrix to be exponentiated.
   * @param rows How many rows there are.
   * @param dst Array where the result will be stored.
   */
  public static native void exp(double[] src, int rows, double[] dst);

  /**
   * Computes the matrix pow.
   *
   * @param src Array of elements of the matrix to be raised to a power.
   * @param rows How many rows there are.
   * @param exponent The exponent.
   * @param dst Array where the result will be stored.
   */
  public static native void pow(double[] src, int rows, double exponent, double[] dst);

  /**
   * Returns true if (A, B) is a stabilizable pair.
   *
   * <p>(A,B) is stabilizable if and only if the uncontrollable eigenvalues of A, if any, have
   * absolute values less than one, where an eigenvalue is uncontrollable if rank(lambda * I - A, B)
   * &lt; n where n is number of states.
   *
   * @param states the number of states of the system.
   * @param inputs the number of inputs to the system.
   * @param A System matrix.
   * @param B Input matrix.
   * @return If the system is stabilizable.
   */
  public static native boolean isStabilizable(int states, int inputs, double[] A, double[] B);

  public static class Helper {
    private static final AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }
}
