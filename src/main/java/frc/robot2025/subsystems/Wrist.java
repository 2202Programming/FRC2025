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
  double cmdVel = 0;

  public enum WristLevels{
    LCoral(75.5),
    LOne(30),
    LTwo(75.5),
    LThree(116),
    LFour(176),
    Ground(0);

    public double height;

    private WristLevels(double height) {
      this.height = height;
    }
  }; 
  public Wrist() {
    wristServo = new NeoServo(CAN.WRIST, anglePositionPID, hwAngleVelPID, true, ClosedLoopSlot.kSlot0);
    wristServo.setConversionFactor(360.0 / AngleGearRatio) // [deg] for internal encoder behind gears
    // .setConversionFactor(360.0) // [deg] external encoder on arm shaft
    .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
    .setVelocityHW_PID(wristMaxVel, wristMaxAccel)
    .setTolerance(wristPosTol, wristVelTol)
    .setMaxVelocity(wristMaxVel)
    .burnFlash();
    wristServo.setClamp(minPos, maxPos);
  }
  public void setWristPosition(WristLevels level){
    wristServo.setPosition(level.height);
  }
public void setWristPosition(double pos){
  wristServo.setPosition(pos);
}
public double getWristPosition(){
  return wristServo.getPosition();
}
public void setWristVelocity (double vel) {
  cmdVel = vel;
  wristServo.setVelocityCmd(vel);
}
public double getVelocity() {
  return wristServo.getVelocity();
} 
public double getCmdVelocity(){
  return cmdVel;
}
public void setWristSetpoint(double pos){
  wristServo.setSetpoint(pos);
}
public void setWristSetpoint(WristLevels wristLevel){
  wristServo.setSetpoint(wristLevel.height);
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
@Override
public void periodic() {
  this.wristServo.periodic();
  // This method will be called once per scheduler run
}

class WristWatcherCmd extends WatcherCmd{ 
  NetworkTableEntry nt_cmdVelocity;
  NetworkTableEntry nt_measVelocity;
  NetworkTableEntry nt_measPosition;
  NetworkTableEntry nt_cmdPosition; //setpoint
  NetworkTableEntry nt_atSetpoint;
    // add nt for pos when we add it
    @Override
  public String getTableName() {
    return Wrist.this.getName();
  }


public void ntcreate() {
  NetworkTable table = getTable();
  nt_cmdVelocity = table.getEntry("cmdVelocity");
  nt_measVelocity = table.getEntry("measVelocity");
  nt_measPosition = table.getEntry("measPosition");
  nt_cmdPosition = table.getEntry("cmdPosition");
  nt_atSetpoint = table.getEntry("atSetpoint");
}

public void ntupdate() {
  nt_cmdVelocity.setDouble(getCmdVelocity());
  nt_measVelocity.setDouble(getVelocity());
  nt_measPosition.setDouble(getWristPosition());
  nt_cmdPosition.setDouble(getWristSetpoint());
  nt_atSetpoint.setBoolean(atSetPoint());
}
}
}
