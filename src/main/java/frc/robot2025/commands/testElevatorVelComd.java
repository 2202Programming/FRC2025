// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;

public class testElevatorVelComd extends Command {
  /** Creates a new testElevatorVelComd. */
   //should be done as a while true command 
  private final Elevator_Subsystem elevator_Subsystem;
  double vel;
  double kP;
  double kI;
  double kD;
  
  public testElevatorVelComd(double vel) {
    SmartDashboard.putNumber("Current Vel", 0.0);
    SmartDashboard.putNumber("kP", 0.1);
    SmartDashboard.putNumber("kI", 0.1);
    SmartDashboard.putNumber("kD", 0.1);
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    SmartDashboard.putNumber("current kP", elevator_Subsystem.getP());
    this.vel = vel;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vel = SmartDashboard.getNumber("Current Vel", 0.0);
    kP = SmartDashboard.getNumber("kP", -0.1);
    kI = SmartDashboard.getNumber("kI", -0.1);
    kD = SmartDashboard.getNumber("kD", -0.1);
    elevator_Subsystem.setVelocity(vel);
    elevator_Subsystem.setP(kP);
    elevator_Subsystem.setI(kI);
    elevator_Subsystem.setD(kD);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_Subsystem.setVelocity(0);
  }

  @Override
  public void execute(){
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
