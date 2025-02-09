// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberSetPos extends Command {
  /** Creates a new ClimberSetPos. */
  final Climber climber;
  double pos;
  public ClimberSetPos(double pos) {
    climber = RobotContainer.getSubsystem(Climber.class);
    this.pos = pos;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setSetpoint(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return climber.atSetpoint();
  }
}
