// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.DropSequenceBaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setWristPos extends Command {
  Wrist wrist;
  double setpoint;
  boolean drop;
  /** Creates a new setWristPos. */
  public setWristPos(boolean drop) {
    wrist = RobotContainer.getSubsystem(Wrist.class);
    this.drop = drop;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(drop){
    wrist.setPos(wrist.drop);
    } else {
      wrist.setPos(wrist.pickup);
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atSetpoint();
  }
}
