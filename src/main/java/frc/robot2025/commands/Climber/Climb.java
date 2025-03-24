// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

/*
   * *********** CLIMBER SEQUENCE PER DEAN
   * ///climb prep
   * 1. Release pin with solenoid (NC) (PDP Switch ID 0)
   * 2. Give motor low or no pwr for REVs
   * 3. Lock pin
   * ///climb
   * 4.wait for DIO to trip (high-low)
   * 5. release pin
   * 6. climb for x REVs
   * 7. stop
   * 
   */
  
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  Climber climber;
  public Climb() {
     climber = RobotContainer.getSubsystem(Climber.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
