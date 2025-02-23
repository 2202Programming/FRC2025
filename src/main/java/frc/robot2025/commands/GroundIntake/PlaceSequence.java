// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.GroundIntake.Position;

public class PlaceSequence extends Command {
  public enum State {
    WaitForPlacePos, // wait for subsystem to get to commanded position
    Eject, // eject coral after arm gets to setpoint
    DefaultPos, // go to zero 
    Finished // command is done
  }

  final int ejectingFrameCount = 5; //tbd

  State state;
  int count;
  final GroundIntake groundIntake;
  final Position place;
  final Position rest;
  final BooleanSupplier  hasPiece;
  final double WheelSpeed;
  
  
  public PlaceSequence(String gp) {
    this(gp, -1.0);
  }

  public PlaceSequence(String gp, double WheelSpeed) {
    this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
    this.WheelSpeed = WheelSpeed;
    if (gp.startsWith("a")){
      place = Position.ALGAE_PLACE;
      rest = Position.ALGAE_REST;
      hasPiece = groundIntake::senseAlgae;
    } else {
      place = Position.CORAL_PLACE;
      rest = Position.CORAL_REST;
      hasPiece = groundIntake::senseCoral;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     count = ejectingFrameCount;
    if (hasPiece.getAsBoolean() ) {  // goes to position without ejecting
      groundIntake.setSetpoint(place);
      //groundIntake.setWheelSpeed(0.0);  should alrady be 0 or a hold cmd
      state = State.WaitForPlacePos;
    } else {
      groundIntake.setSetpoint(Position.ZERO);
      state = State.DefaultPos;
    }
  }

  @Override
  public void execute() {
    switch (state) {

      case WaitForPlacePos:
        if (groundIntake.isAtSetpoint()) {
          state = State.Eject; 
          groundIntake.setWheelSpeed(WheelSpeed);      
        }
        break;

      case Eject:
         
        if (count <= 0 && !hasPiece.getAsBoolean()) { 
          groundIntake.setSetpoint(Position.ZERO);
          groundIntake.setWheelSpeed(0.0);
          state = State.DefaultPos;
        }
        break;

      case DefaultPos:
        if (groundIntake.isAtSetpoint() ) {
          state = State.Finished; 
        }
        break;

      case Finished:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (hasPiece.getAsBoolean()) {
      groundIntake.setSetpoint(rest);
      groundIntake.setWheelSpeed(0.0);
    } else {
      groundIntake.setSetpoint(Position.ZERO);
      groundIntake.setWheelSpeed(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == State.Finished;
  }
}
