// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
  
  final Elevator_Subsystem elevator_Subsystem;
  double intendedPoint;
  Levels setPoint;

  public ElevatorMove(Levels setPoint) {
    this.elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.setPoint = setPoint;
    intendedPoint = setPoint.height;
    // this command require full ownership of SS, use the requirements
    addRequirements(this.elevator_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_Subsystem.setHeight(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      elevator_Subsystem.setHeight(Levels.PickUp);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator_Subsystem.atSetpoint(); //change depending on acceptable error
  }
}
