// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.GroundIntake.Position;

public class CoralPickupSequence extends Command {

  public enum State{
    WaitForCoral, // wait until coral trips lightgate
    CoralRest, // sets position to transport coral
    Finished // sequence is finished, either no coral was picked up or its ready for transport
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
    groundIntake.setPosition(Position.CORAL_PICKUP);
    groundIntake.setWheelSpeed(1.0); //placeholder
    state = State.WaitForCoral;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state){

      case WaitForCoral:
        hasCoral = groundIntake.senseGamePiece();
        state = hasCoral ? State.CoralRest : State.WaitForCoral;
        break;
      
      case CoralRest:
        groundIntake.setPosition(Position.CORAL_REST);
        groundIntake.setWheelSpeed(0.0);
        state = State.Finished;
        break;

      case Finished:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntake.setPosition(groundIntake.senseGamePiece() ? Position.CORAL_REST : Position.ZERO);
    groundIntake.setWheelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == State.Finished;
  }
}
