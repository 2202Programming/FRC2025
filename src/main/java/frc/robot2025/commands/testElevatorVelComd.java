// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;

public class testElevatorVelComd extends Command {
  /** Creates a new testElevatorVelComd. */
   //should be done as a while true command 
  private final Elevator_Subsystem elevator_Subsystem;
  double speed;
  
  public testElevatorVelComd(double speed) {
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_Subsystem.set_speed(speed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_Subsystem.set_speed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
