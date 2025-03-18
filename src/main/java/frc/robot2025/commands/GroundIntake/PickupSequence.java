// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.GroundIntake.Position;

public class PickupSequence extends Command {
  // used to pick gamepiece up from the floor and then move to traveling position (rest) if gamepiece is obtained. if not, keep going
  // until driver lets go of button -er
  public enum State{
    WaitForGamepiece, // wait until coral trips limitswitch
    Rest, // sets position to transport coral
    WaitForMove,
    Finished, // sequence is finished, either no coral was picked up or its ready for transport
    WaitForPickupPos // wait for system to get to setpoint
  }
  State state;
  public static final int FrameCount = 3;
  final GroundIntake groundIntake;
  final Position pickup;
  final Position rest;
  final BooleanSupplier hasPiece;
  int pickupFrameCounter;
  final double holdVolts;
  final double holdAngle;


  public PickupSequence(String gp) {
  this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
    if(gp.startsWith("a")){
      pickup = Position.ALGAE_PICKUP;
      rest = Position.ALGAE_REST;
      hasPiece = groundIntake::senseAlgae;
      holdVolts = 1.0;
      holdAngle = 0.0;
    } else {
      pickup = Position.CORAL_PICKUP;
      rest = Position.CORAL_REST;
      hasPiece = groundIntake::senseCoral;
      holdVolts = 0.0;
      holdAngle = 5.0; // pos num closes -er
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickupFrameCounter = 0; 
    groundIntake.setSetpoint(pickup);
    groundIntake.setWheelSpeed(50.0); 
    state = State.WaitForPickupPos;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state){
      
      case WaitForPickupPos:
        state = groundIntake.isAtSetpoint() ? State.WaitForGamepiece : State.WaitForPickupPos;
        break;

      case WaitForGamepiece: 
        state = hasPiece.getAsBoolean() && pickupFrameCounter++ == FrameCount ? State.Rest : State.WaitForGamepiece;
        break;
      
      case Rest:
        groundIntake.setSetpoint(rest);
        //groundIntake.setWheelSpeed(0.0); //Shouldn't need if holding 
        groundIntake.hold(holdAngle);
        groundIntake.setWheelHold(holdVolts);
        state = State.WaitForMove;
        break;
      
      case WaitForMove:
        state = groundIntake.isBottomAtSetpoint() ? State.Finished : State.WaitForMove;
        break;

      case Finished:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("pickup sequence interrupted");
    groundIntake.setSetpoint(hasPiece.getAsBoolean() ? rest : Position.ZERO);
    if (hasPiece.getAsBoolean())
    {
      groundIntake.setWheelHold(holdVolts);
    } else {
      groundIntake.setWheelSpeed(0.0); 
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == State.Finished;
  }
}
