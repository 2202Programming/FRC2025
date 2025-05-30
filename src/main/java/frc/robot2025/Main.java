// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib2202.builder.Robot;
import frc.lib2202.builder.RobotSpecDefault;

public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    new RobotSpecDefault();
    new RobotSpec_BetaBot2025();
    new RobotSpec_AlphaBot2025();
    new RobotSpec_BotOnBoard();
    new RobotSpec_BotOnBoard2();
    new RobotSpec_BotOnBoard3();
    new RobotSpec_test2024();      //2024 platform, 2025 limelight and paths
  }
 
  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}
