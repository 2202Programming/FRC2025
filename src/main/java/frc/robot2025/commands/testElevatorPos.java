// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;

public class testElevatorPos extends Command {
  /** Creates a new testElevatorPos. */

  double pos;
  Elevator_Subsystem elevator_Subsystem;

  public testElevatorPos(double pos) {
    this.elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_Subsystem.set_toHeight(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator_Subsystem.at_setpoint();
  }
}
