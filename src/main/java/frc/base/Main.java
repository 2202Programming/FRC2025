// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.base;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib2202.builder.RobotSpecDefault;
import frc.robot2025.RobotSpec_AlphaBot2025;

public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    new RobotSpecDefault();
    new RobotSpec_AlphaBot2025();


  }
 
  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}
