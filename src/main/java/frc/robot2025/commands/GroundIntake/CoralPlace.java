// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.GroundIntake.Position;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPlace extends Command {
  public enum State{
    Eject, DefaultPos, Finished
  }
  State state;
  final GroundIntake groundIntake;
  boolean hasCoral;
    
      public CoralPlace() {
        this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
        this.hasCoral = groundIntake.senseGamePiece();
      }
      
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        if(hasCoral == true){
          groundIntake.setGroundIntakePosition(Position.CORAL_PLACE);
          groundIntake.setGroundIntakeWheelSpeed(0.0);
          if(groundIntake.isAtSetpoint()){
            state = State.Eject;
          }
        } else {
          state = State.DefaultPos;
        }
      }
    
      @Override
      public void execute() {
        switch(state){
        
          case Eject:
            groundIntake.setGroundIntakeWheelSpeed(-1.0);
            hasCoral = groundIntake.senseGamePiece();
            if(!hasCoral){
              state = State.DefaultPos;
            }
            break;
    
          case DefaultPos:
          groundIntake.setGroundIntakePosition(Position.ZERO);
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
        if(hasCoral == true){
          groundIntake.setGroundIntakePosition(Position.CORAL_REST);
          groundIntake.setGroundIntakeWheelSpeed(0.0);      
        } else {
          groundIntake.setGroundIntakePosition(Position.ZERO);
          groundIntake.setGroundIntakeWheelSpeed(0.0);
        }
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return state == State.Finished;
  }
}
