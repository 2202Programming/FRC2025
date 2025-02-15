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

public class backupEE extends SubsystemBase {
  NeoServo servo;
  PIDController backupEEPID = new PIDController(0, 0, 0);
  PIDFController hwbackupEEVel_PID = new PIDFController(0, 0, 0, 0);

  final double GearRatio = 50.0 * 10.0;
  final double conversionFactor = 1.0 / GearRatio; // placeholder
  // Motor settings for Servo
  final int STALL_CURRENT = 20;
  final int FREE_CURRENT = 40;
  final boolean motor_inverted = true;
  // Servo speed/positions
  final double maxVel = 50.0; // [unit?/s] placeholder
  final double maxAccel = 5.0; // placeholder
  final double velTol = 1.0; // placeholder

  double cmdVel; //

  /** Creates a new backupEE. */
  public backupEE() {
    servo = new NeoServo(Constants.CAN.END_EFFECTOR, backupEEPID, hwbackupEEVel_PID, motor_inverted);
    servo.setConversionFactor(conversionFactor)
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setMaxVelocity(maxVel);

    // initial position, unless we have abs encoder
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

  public double getMaxVel() {
    return servo.getMaxVel();
  }

  public void setMaxVelocity(double vel) {
    servo.setMaxVelocity(vel);
  }

  public double getCmdVelocity() {
    return cmdVel;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.servo.periodic();
  }

  class backupEEWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdVelocity;
    NetworkTableEntry nt_measVelocity;
    // add nt for pos when we add it

    @Override
    public String getTableName() {
      return "backupEE";
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdVelocity = table.getEntry("cmdVelocity");
      nt_measVelocity = table.getEntry("measVelocity");

    }

    public void ntupdate() {
      nt_cmdVelocity.setDouble(getCmdVelocity());
      nt_measVelocity.setDouble(getVelocity());
    }
  }
}
