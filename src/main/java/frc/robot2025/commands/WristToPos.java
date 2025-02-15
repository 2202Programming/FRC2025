// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristToPos extends InstantCommand {
  Wrist servo = RobotContainer.getSubsystem(Wrist.class);
  double pos;

  public WristToPos(double pos) {
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    servo.setPos(pos);
  }
}
