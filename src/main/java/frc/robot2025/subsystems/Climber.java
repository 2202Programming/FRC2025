// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2025.Constants;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

public class Climber extends SubsystemBase {
  NeoServo servo;
  PIDFController hwClimberVel_PID = new PIDFController(0.01, 0, 0, 1 / 540);

  final double GearRatio = 9.0 * 5.0 * 4.0;
  final double conversionFactor = 1.0 / GearRatio;

ClimberWatcherCmd watcher;

  // Motor settings for Servo
  final int STALL_CURRENT = 40;
  final int FREE_CURRENT = 40;
  final boolean motor_inverted = true;
  // Servo speed/positions
  final double maxVel = 0.75;  // [winch rot/s]
  final double maxAccel = 0.75; // [winch rot/s/s]
  final double posTol = 0.01; // tol = tolerance [rot]
  final double velTol = 0.1; // [rot/s]

  double cmdVel;

  /** Creates a new Climber. */
  public Climber() {
    servo = new NeoServo(Constants.CAN.CLIMBER, new PIDController(0 ,0, 0), hwClimberVel_PID, motor_inverted);
    servo.setConversionFactor(conversionFactor)
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel);
        this.watcher = new ClimberWatcherCmd();

        watcher.ntcreate();
  }

  // velocity control only used for testing, normal cmds will use position
  public void setVelocity(double vel) {
    cmdVel = vel;
    servo.setVelocityCmd(vel);
  }

  public double getVelocity() {
    return servo.getVelocity();
  }

  public boolean atVelocity() {
    return Math.abs(getVelocity() - cmdVel) < velTol;
  }

  public double getPosition() {
    return servo.getPosition();
  }

  public double getSetpoint() {
    return servo.getSetpoint();
  }

  public void setSetpoint(double pos) {
    servo.setSetpoint(pos);
  }

  public double getMaxVel() {
    return servo.getMaxVel();
  }

  public void setMaxVelocity(double vel) {
    servo.setMaxVelocity(vel);
  }

  public double getCmdVelocity() {
    return cmdVel;
  }

  public boolean atSetpoint(){
    return servo.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.servo.periodic();

    watcher.ntupdate();
  }

  class ClimberWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdVelocity;
    NetworkTableEntry nt_measVelocity;
    NetworkTableEntry nt_measPosition;
    NetworkTableEntry nt_cmdPosition; // setpoint
    NetworkTableEntry nt_atSetpoint;
    // add nt for pos when we add it

    @Override
    public String getTableName() {
      return "Climber";
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdVelocity = table.getEntry("cmdVelocity");
      nt_measVelocity = table.getEntry("measVelocity");
      nt_measPosition = table.getEntry("measPosition");
      nt_cmdPosition = table.getEntry("cmdPosition");
    }

    public void ntupdate() {
      nt_cmdVelocity.setDouble(getCmdVelocity());
      nt_measVelocity.setDouble(getVelocity());
      nt_measPosition.setDouble(getPosition());
      nt_cmdPosition.setDouble(getSetpoint());
    }
  }
}
