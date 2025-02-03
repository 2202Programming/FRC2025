// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.GroundIntake.Position;
import frc.lib2202.builder.RobotContainer;

public class AlgaePickupSequence extends Command {

  public enum State {
    WaitForAlgae, // wait for lg to trip
    AlgaeRest, // when algae is in robot for transport to processor
    Finished // sequence is finished, gamepiece is in position for transport
  }
  State state;
  final GroundIntake groundIntake;
  boolean hasAlgae;

  public AlgaePickupSequence() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
    this.hasAlgae = false;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundIntake.setGroundIntakePosition(Position.ALGAE_PICKUP);
    groundIntake.setGroundIntakeWheelSpeed(1.0); // placeholder
    state = State.WaitForAlgae;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state){

      case WaitForAlgae:
        hasAlgae = groundIntake.senseGamePiece();
        state = hasAlgae ? State.AlgaeRest : State.WaitForAlgae;
        break;

      case AlgaeRest:
        groundIntake.setGroundIntakePosition(Position.ALGAE_REST);
        groundIntake.setGroundIntakeWheelSpeed(0.0);
        state = State.Finished;
        break;

      case Finished:
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntake.setGroundIntakePosition(groundIntake.senseGamePiece() ? Position.ALGAE_REST : Position.ZERO);
    groundIntake.setGroundIntakeWheelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == State.Finished;
  }
}
