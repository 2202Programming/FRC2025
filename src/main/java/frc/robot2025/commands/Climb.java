// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;
import frc.robot2025.subsystems.Elevator_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberSetPos extends Command {
  /** Creates a new ClimberSetPos. */
  final Elevator_Subsystem elevator_Subsystem;
  double pos;
  double desired;
  public ClimberSetPos(double pos) {
    SmartDashboard.putNumber("Desired Pos", 0.0);
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.pos = pos;
  }

  public Climb(double vel) {
    // 3.0 tested on test stand, good default value; re-tune when needed
    this(3.0, vel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desired = SmartDashboard.getNumber("Desired Pos", 0.0);
    elevator_Subsystem.setHeight(desired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return elevator_Subsystem.atSetpoint();
  }
}
