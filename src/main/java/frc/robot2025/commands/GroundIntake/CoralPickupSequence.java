// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2025.subsystem.GroundIntake.Position;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPickupSequence extends Command {

  public enum State {
    WaitForCoral,
    CoralRest,
    Finished
  }
  State state;
  final GroundIntake groundIntake;
  boolean hasCoral;

  public CoralPickupSequence() {
    this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
    this.hasCoral = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundIntake.setGroundIntakePosition(Position.CORAL_PICKUP);
    groundIntake.setGroundIntakeWheelSpeed(1.0);
    state = State.WaitForCoral;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case WaitForCoral:
        break;

      case CoralRest:
        break;

      case Finished:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
