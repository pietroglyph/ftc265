/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.numbers;


import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Nat;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Num;

/**
 * A class representing the number 0.
*/
public final class N0 extends Num implements Nat<N0> {
  private N0() {
  }

  /**
   * The integer this class represents.
   *
   * @return The literal number 0.
  */
  @Override
  public int getNum() {
    return 0;
  }

  /**
   * The singleton instance of this class.
  */
  public static final N0 instance = new N0();
}
