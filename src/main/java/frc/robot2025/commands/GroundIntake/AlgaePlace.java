// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.GroundIntake.Position;

public class AlgaePlace extends Command {
  public enum State {
    WaitForAlgaePlacePos, // wait for subsystem to get to commanded position
    Eject, // eject algae after arm gets to setpoint
    DefaultPos, // go to zero
    Finished // command is done
  }

  State state;
  final GroundIntake groundIntake;
  boolean hasAlgae;

  public AlgaePlace() {
    this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
    hasAlgae = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (hasAlgae == true) {
      groundIntake.setSetpoint(Position.ALGAE_PLACE);
      groundIntake.setWheelSpeed(0.0);
      state = State.WaitForAlgaePlacePos;
    } else {
      state = State.DefaultPos;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {

      case WaitForAlgaePlacePos:
        if (groundIntake.isAtSetpoint()) {
          state = State.Eject;
        }
        break;

      case Eject:
        groundIntake.setWheelSpeed(-1.0);
        hasAlgae = groundIntake.senseGamePiece();
        if (!hasAlgae) {
          state = State.DefaultPos;
        }
        break;

      case DefaultPos:
        groundIntake.setSetpoint(Position.ZERO);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
