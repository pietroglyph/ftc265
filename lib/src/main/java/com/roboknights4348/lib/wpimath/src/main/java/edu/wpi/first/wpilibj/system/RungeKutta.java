// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.system;

import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Matrix;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Num;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.numbers.N1;

import java.util.function.BiFunction;
import java.util.function.DoubleFunction;
import java.util.function.Function;


public final class RungeKutta {
  private RungeKutta() {
    // utility Class
  }

  /**
   * Performs Runge Kutta integration (4th order).
   *
   * @param f The function to integrate, which takes one argument x.
   * @param x The initial value of x.
   * @param dtSeconds The time over which to integrate.
   * @return the integration of dx/dt = f(x) for dt.
   */
  @SuppressWarnings("ParameterName")
  public static double rungeKutta(DoubleFunction<Double> f, double x, double dtSeconds) {
    final double halfDt = 0.5 * dtSeconds;
    final double k1 = f.apply(x);
    final double k2 = f.apply(x + k1 * halfDt);
    final double k3 = f.apply(x + k2 * halfDt);
    final double k4 = f.apply(x + k3 * dtSeconds);
    return x + dtSeconds / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

  /**
   * Performs Runge Kutta integration (4th order).
   *
   * @param f The function to integrate. It must take two arguments x and u.
   * @param x The initial value of x.
   * @param u The value u held constant over the integration period.
   * @param dtSeconds The time over which to integrate.
   * @return The result of Runge Kutta integration (4th order).
   */
  @SuppressWarnings("ParameterName")
  public static double rungeKutta(
      BiFunction<Double, Double, Double> f, double x, Double u, double dtSeconds) {
    final double halfDt = 0.5 * dtSeconds;
    final double k1 = f.apply(x, u);
    final double k2 = f.apply(x + k1 * halfDt, u);
    final double k3 = f.apply(x + k2 * halfDt, u);
    final double k4 = f.apply(x + k3 * dtSeconds, u);
    return x + dtSeconds / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

  /**
   * Performs 4th order Runge-Kutta integration of dx/dt = f(x, u) for dt.
   *
   * @param <States> A Num representing the states of the system to integrate.
   * @param <Inputs> A Num representing the inputs of the system to integrate.
   * @param f The function to integrate. It must take two arguments x and u.
   * @param x The initial value of x.
   * @param u The value u held constant over the integration period.
   * @param dtSeconds The time over which to integrate.
   * @return the integration of dx/dt = f(x, u) for dt.
   */
  @SuppressWarnings({"ParameterName", "MethodTypeParameterName"})
  public static <States extends Num, Inputs extends Num> Matrix<States, N1> rungeKutta(
      BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
      Matrix<States, N1> x,
      Matrix<Inputs, N1> u,
      double dtSeconds) {

    final double halfDt = 0.5 * dtSeconds;
    Matrix<States, N1> k1 = f.apply(x, u);
    Matrix<States, N1> k2 = f.apply(x.plus(k1.times(halfDt)), u);
    Matrix<States, N1> k3 = f.apply(x.plus(k2.times(halfDt)), u);
    Matrix<States, N1> k4 = f.apply(x.plus(k3.times(dtSeconds)), u);
    return x.plus((k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4)).times(dtSeconds).div(6.0));
  }

  /**
   * Performs 4th order Runge-Kutta integration of dx/dt = f(x) for dt.
   *
   * @param <States> A Num prepresenting the states of the system.
   * @param f The function to integrate. It must take one argument x.
   * @param x The initial value of x.
   * @param dtSeconds The time over which to integrate.
   * @return 4th order Runge-Kutta integration of dx/dt = f(x) for dt.
   */
  @SuppressWarnings({"ParameterName", "MethodTypeParameterName"})
  public static <States extends Num> Matrix<States, N1> rungeKutta(
          Function<Matrix<States, N1>, Matrix<States, N1>> f, Matrix<States, N1> x, double dtSeconds) {

    final double halfDt = 0.5 * dtSeconds;
    Matrix<States, N1> k1 = f.apply(x);
    Matrix<States, N1> k2 = f.apply(x.plus(k1.times(halfDt)));
    Matrix<States, N1> k3 = f.apply(x.plus(k2.times(halfDt)));
    Matrix<States, N1> k4 = f.apply(x.plus(k3.times(dtSeconds)));
    return x.plus((k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4)).times(dtSeconds).div(6.0));
  }
}
