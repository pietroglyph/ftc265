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
 * A class representing the number 19.
*/
public final class N19 extends Num implements Nat<N19> {
  private N19() {
  }

  /**
   * The integer this class represents.
   *
   * @return The literal number 19.
  */
  @Override
  public int getNum() {
    return 19;
  }

  /**
   * The singleton instance of this class.
  */
  public static final N19 instance = new N19();
}
