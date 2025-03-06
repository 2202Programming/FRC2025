// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.robot2025.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPlaceSequence extends Command {
  /** Creates a new EE_Release. */
    final EndEffector_Subsystem ee_Subsystem;
    final Wrist wrist;
    final Elevator_Subsystem elevator_Subsystem;
    double pos;
    enum Phase {
      moveElev, wristSet, Drop, wristReset, pickupPos, finished
    }
    Phase phase;
  public CoralPlaceSequence(double pos) {
    ee_Subsystem = RobotContainer.getSubsystem(EndEffector_Subsystem.class);
    wrist = RobotContainer.getSubsystem(Wrist.class);
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    phase = Phase.moveElev;
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!ee_Subsystem.pieceReady()){
      phase = Phase.finished;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase){
      case moveElev:
      System.out.println("phase1");
      elevator_Subsystem.setHeight(pos);
      if(elevator_Subsystem.atSetpoint()){
        phase = Phase.wristSet;
      }
      break;
      case wristSet:
      System.out.println("phase2");
        wrist.setPos(wrist.drop);
        if(wrist.atSetpoint()){
          phase = Phase.Drop;
        }
        break;
      case Drop:
      System.out.println("phase3");
        ee_Subsystem.setPercent(-0.6);
        if(!ee_Subsystem.pieceReady()){
          phase = Phase.wristReset;
        }
        break;
      case wristReset:
      System.out.println("phase4");
        wrist.setPos(wrist.pickup);
        if(wrist.atSetpoint()){
          phase = Phase.pickupPos;
        }
        break;
      case pickupPos:
      System.out.println("phase5");
        elevator_Subsystem.setHeight(50.0);
        if(elevator_Subsystem.atSetpoint()){
          phase = Phase.finished;
        }
        break;
      case finished:
      default:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.finished;
  }
}
