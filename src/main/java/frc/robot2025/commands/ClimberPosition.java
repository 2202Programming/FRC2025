package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

public class ClimberPosition extends Command {
  final Climber climber;
  final double maxVel;
  double desired;

  public ClimberPosition(double maxVel) {
    // 3.0 tested on test stand, good default value; re-tune when needed
    this(3.0, maxVel);
  }

  public ClimberPosition(double pos, double maxVel) {
    SmartDashboard.putNumber("Desired Pos", pos);
    climber = RobotContainer.getSubsystem(Climber.class);
    this.maxVel = maxVel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desired = SmartDashboard.getNumber("Desired Pos", 0.0);
    climber.setMaxVelocity(maxVel);
    climber.setSetpoint(desired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return climber.atSetpoint();
  }
}
