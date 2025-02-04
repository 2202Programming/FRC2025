// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib2202.command.WatcherCmd;

public class Wrist extends SubsystemBase {
  /** Creates a new wrist. */
  NeoServo wristServo;
    PIDFController hwAngleVelPID = new PIDFController(0.0050, 0.0, 0.0, 0.0075);
  final PIDController anglePositionPID = new PIDController(7.0, 1.0, 0.0); 
  final double AngleGearRatio = 1.0; // placeholder
  final int STALL_CURRENT = 20;
  final int FREE_CURRENT = 40;
  final int wristMaxVel = 50; // [deg/s]
  final int wristMaxAccel = 10; // [deg/s^2]
  final int wristPosTol = 1;
  final int wristVelTol = 1;
  final int maxPos = 100;
  final int minPos = -10;
  public Wrist() {
    wristServo = new NeoServo(CAN.AMP_MECHANISM, anglePositionPID, hwAngleVelPID, true, ClosedLoopSlot.kSlot0);
    wristServo.setConversionFactor(360.0 / AngleGearRatio) // [deg] for internal encoder behind gears
    // .setConversionFactor(360.0) // [deg] external encoder on arm shaft
    .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
    .setVelocityHW_PID(wristMaxVel, wristMaxAccel)
    .setTolerance(wristPosTol, wristVelTol)
    .setMaxVelocity(wristMaxVel)
    .burnFlash();
    wristServo.setClamp(minPos, maxPos);
  }
public void setWristPosition(double pos){
  wristServo.setPosition(pos);
}
public double getWristPosition(){
  return wristServo.getPosition();
}
public void setWristVelocity (double vel) {
  wristServo.setVelocityCmd(vel);
}
public double getVelocity() {
  return wristServo.getVelocity();
} 
public void setWristSetpoint(double pos){
  wristServo.setSetpoint(pos);
}
public double getWristSetpoint() {
  return wristServo.getSetpoint();
}
public void setMaxVelocity(double vel) {
  wristServo.setMaxVelocity(vel);
}
public boolean atSetPoint(){
  return Math.abs(getWristPosition() - getWristSetpoint()) < wristPosTol;

}

class WristWatcherCmd extends WatcherCmd{ 
  NetworkTableEntry nt_cmdRPM;
  NetworkTableEntry nt_measRPM;
  NetworkTableEntry nt_kP;
  NetworkTableEntry nt_kF;
}

public String getTableName() {
  return Wrist.this.getName();
}

public void ntcreate() {
  NetworkTable table = getTable();
  nt_cmdRPM = table.getEntry("cmdRPM");
  nt_measRPM = table.getEntry("measRPM");
  nt_kP = table.getEntry("kP");
  nt_kF = table.getEntry("kF");
}

public void ntupdate() {
  nt_cmdRPM.setDouble(cmdRPM);
  nt_measRPM.setDouble(measRPM);
  nt_kP.setDouble(pid.getP());
  nt_kF.setDouble(pid.getF());
}
  @Override
  public void periodic() {
    this.wristServo.periodic();
    // This method will be called once per scheduler run
  }
}
