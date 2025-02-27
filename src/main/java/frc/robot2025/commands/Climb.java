// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {

  Climber climber;
  double pos;
  double vel;
  double tolerance = 0.01; //[rot]

  /** Creates a new ClimberMoveTo. */
  public Climb(double pos, double vel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    climber = RobotContainer.getSubsystem(Climber.class);

    this.pos = pos;
    this.vel = vel;
  }

  public Climb(double vel) {
    // 3.0 tested on test stand, good default value; re-tune when needed
    this(3.0, vel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setVelocity(vel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.getPosition() - pos) < tolerance;
  }
}
