// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.base;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib2202.builder.RobotSpecDefault;
import frc.robot2025.RobotSpec_SwerveBot;

public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    new RobotSpecDefault();
    new RobotSpec_SwerveBot();


  }
  public static String serialnum="032D2062"; // robot2024 comp bot

  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}