// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Wrist;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.Wrist.WristLevels;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PickupSequence extends Command {
  Elevator_Subsystem elevator;
  Wrist wrist;
  EndEffector_Subsystem endEffector;
  Levels level;
  WristLevels wristLevel;
  final int DELAY_COUNT = 25;
  int count;

  /** Creates a new PickupSequence. */
  public enum Phase {
    ElevatorInPos, WaitingForCoral, Finished
  }

  Phase phase;

  public PickupSequence(Levels level, WristLevels wristLevel) {
    elevator = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    wrist = RobotContainer.getSubsystem(Wrist.class);
    endEffector = RobotContainer.getSubsystem(EndEffector_Subsystem.class);
    this.level = level;
    this.wristLevel = wristLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    //systwem.out.println("sup, from Avdhut");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = Phase.ElevatorInPos;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(phase){
      case ElevatorInPos:
        elevator.setHeight(level);
        wrist.setWristSetpoint(wristLevel);
        if(elevator.atSetpoint() && wrist.atSetPoint()){
          wrist.setWristVelocity(0);
          elevator.setVelocity(0);
          phase = Phase.WaitingForCoral;
        }
        break;
      case WaitingForCoral:
        endEffector.setRPM(-500); //placeholder
        if(endEffector.hasPiece()){
          count++;
        }
        if(count >= DELAY_COUNT){
          phase = Phase.Finished;
        }
        break;
      default:
      break;
        }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setWristSetpoint(WristLevels.LFour);
    elevator.setHeight(Levels.LFour);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
