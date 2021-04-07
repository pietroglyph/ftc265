// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.math;

import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Matrix;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Nat;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Num;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Pair;

import org.ejml.simple.SimpleMatrix;


@SuppressWarnings({"ParameterName", "MethodTypeParameterName"})
public final class Discretization {
  private Discretization() {
    // Utility class
  }

  /**
   * Discretizes the given continuous A matrix.
   *
   * @param <States> Num representing the number of states.
   * @param contA Continuous system matrix.
   * @param dtSeconds Discretization timestep.
   * @return the discrete matrix system.
   */
  public static <States extends Num> Matrix<States, States> discretizeA(
      Matrix<States, States> contA, double dtSeconds) {
    return contA.times(dtSeconds).exp();
  }

  /**
   * Discretizes the given continuous A and B matrices.
   *
   * <p>Rather than solving a (States + Inputs) x (States + Inputs) matrix exponential like in
   * DiscretizeAB(), we take advantage of the structure of the block matrix of A and B.
   *
   * <p>1) The exponential of A*t, which is only N x N, is relatively cheap. 2) The upper-right
   * quarter of the (States + Inputs) x (States + Inputs) matrix, which we can approximate using a
   * taylor series to several terms and still be substantially cheaper than taking the big
   * exponential.
   *
   * @param states Nat representing the states of the system.
   * @param contA Continuous system matrix.
   * @param contB Continuous input matrix.
   * @param dtseconds Discretization timestep.
   */
  public static <States extends Num, Inputs extends Num>
  Pair<Matrix<States, States>, Matrix<States, Inputs>> discretizeABTaylor(
          Nat<States> states,
          Matrix<States, States> contA,
          Matrix<States, Inputs> contB,
          double dtseconds) {
    Matrix<States, States> lastTerm = Matrix.eye(states);
    double lastCoeff = dtseconds;

    Matrix<States, States> phi12 = lastTerm.times(lastCoeff);

    // i = 6 i.e. 5th order should be enough precision
    for (int i = 2; i < 6; ++i) {
      lastTerm = contA.times(lastTerm);
      lastCoeff *= dtseconds / ((double) i);

      phi12 = phi12.plus(lastTerm.times(lastCoeff));
    }

    Matrix<States, Inputs> discB = phi12.times(contB);

    Matrix<States, States> discA = discretizeA(contA, dtseconds);

    return Pair.of(discA, discB);
  }

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * <p>Rather than solving a 2N x 2N matrix exponential like in DiscretizeQ() (which is expensive),
   * we take advantage of the structure of the block matrix of A and Q.
   *
   * <p>The exponential of A*t, which is only N x N, is relatively cheap. 2) The upper-right quarter
   * of the 2N x 2N matrix, which we can approximate using a taylor series to several terms and
   * still be substantially cheaper than taking the big exponential.
   *
   * @param <States> Nat representing the number of states.
   * @param contA Continuous system matrix.
   * @param contQ Continuous process noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return a pair representing the discrete system matrix and process noise covariance matrix.
   */
  @SuppressWarnings("LocalVariableName")
  public static <States extends Num>
      Pair<Matrix<States, States>, Matrix<States, States>> discretizeAQTaylor(
          Matrix<States, States> contA, Matrix<States, States> contQ, double dtSeconds) {
    Matrix<States, States> Q = (contQ.plus(contQ.transpose())).div(2.0);

    Matrix<States, States> lastTerm = Q.copy();
    double lastCoeff = dtSeconds;

    // A^T^n
    Matrix<States, States> Atn = contA.transpose();
    Matrix<States, States> phi12 = lastTerm.times(lastCoeff);

    // i = 6 i.e. 6th order should be enough precision
    for (int i = 2; i < 6; ++i) {
      lastTerm = contA.times(-1).times(lastTerm).plus(Q.times(Atn));
      lastCoeff *= dtSeconds / ((double) i);

      phi12 = phi12.plus(lastTerm.times(lastCoeff));

      Atn = Atn.times(contA.transpose());
    }

    Matrix<States, States> discA = discretizeA(contA, dtSeconds);
    Q = discA.times(phi12);

    // Make Q symmetric if it isn't already
    Matrix<States, States> discQ = Q.plus(Q.transpose()).div(2.0);

    return new Pair<>(discA, discQ);
  }

  /**
   * Returns a discretized version of the provided continuous measurement noise covariance matrix.
   * Note that dt=0.0 divides R by zero.
   *
   * @param <O> Nat representing the number of outputs.
   * @param R Continuous measurement noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return Discretized version of the provided continuous measurement noise covariance matrix.
   */
  public static <O extends Num> Matrix<O, O> discretizeR(Matrix<O, O> R, double dtSeconds) {
    return R.div(dtSeconds);
  }

  /**
   * Discretizes the given continuous A and B matrices.
   *
   * @param <States> Nat representing the states of the system.
   * @param <Inputs> Nat representing the inputs to the system.
   * @param contA Continuous system matrix.
   * @param contB Continuous input matrix.
   * @param dtSeconds Discretization timestep.
   * @return a Pair representing discA and diskB.
   */
  @SuppressWarnings("LocalVariableName")
  public static <States extends Num, Inputs extends Num>
      Pair<Matrix<States, States>, Matrix<States, Inputs>> discretizeAB(
          Matrix<States, States> contA, Matrix<States, Inputs> contB, double dtSeconds) {
    Matrix<States, States> scaledA = contA.times(dtSeconds);
    Matrix<States, Inputs> scaledB = contB.times(dtSeconds);

    int contSize = contB.getNumRows() + contB.getNumCols();
    Matrix Mcont = new Matrix<>(new SimpleMatrix(contSize, contSize));
    Mcont.assignBlock(0, 0, scaledA);
    Mcont.assignBlock(0, scaledA.getNumCols(), scaledB);
    Matrix Mdisc = Mcont.exp();

    Matrix<States, States> discA =
        new Matrix<States, States>(new SimpleMatrix(contB.getNumRows(), contB.getNumRows()));
    Matrix<States, Inputs> discB =
        new Matrix<States, Inputs>(new SimpleMatrix(contB.getNumRows(), contB.getNumCols()));

    discA.extractFrom(0, 0, Mdisc);
    discB.extractFrom(0, contB.getNumRows(), Mdisc);

    return new Pair<>(discA, discB);
  }
}
