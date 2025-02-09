// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristSetPos extends Command {
  /** Creates a new ClimberSetPos. */
  final Wrist wrist;
  double pos;
  public WristSetPos(double pos) {
    wrist = RobotContainer.getSubsystem(Wrist.class);
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setWristSetpoint(pos);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //wrist.setWristVelocity(0);  no need, position servo is running for wrist
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atSetPoint();
  }
}
