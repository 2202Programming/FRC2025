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
  NeoServo climberServo;
  PIDController climberPID = new PIDController(0, 0, 0);
  PIDFController hwClimberVel_PID = new PIDFController(0,0, 0, 0);
  final double GearRatio = 48 * 18 * 18;
  final double conversionFactor = 0.0; //placeholder
  final int STALL_CURRENT = 20;
  final int FREE_CURRENT = 40;
  final double maxVel = 50; //placeholder
  final double maxPos = 100; //placeholder
  final double minPos = 0; //placeholder
  final double maxAccel = 5; //placeholder
  final double posTol = 1; //placeholder
  final double velTol = 1; //placeholder,
  double cmdVel;
  /** Creates a new Climber. */
  public Climber() {
    climberServo = new NeoServo(Constants.CAN.CLIMBER, climberPID, hwClimberVel_PID, true);
    climberServo.setConversionFactor(conversionFactor)
    .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
    .setVelocityHW_PID(maxVel, maxAccel)
    .setTolerance(posTol, velTol)
    .setMaxVelocity(maxVel)
    .burnFlash();
    climberServo.setClamp(minPos, maxPos);
    
  }

  public void setVelocity(double vel) {
    cmdVel = vel;
    climberServo.setVelocityCmd(vel);
  }


  public double getVelocity() {
    return climberServo.getVelocity(); 
  }

  public boolean atVelocity() {
    return Math.abs(getVelocity() - cmdVel) < velTol;
  }
  
  public void setPosition(double pos){
    climberServo.setPosition(pos);
  }

  public double getPosition(){
    return climberServo.getPosition();
  }
  public double getSetpoint(){
    return climberServo.getSetpoint();
  }
  public void setSetpoint(double pos){
     climberServo.setSetpoint(pos);
  }
  public double getMaxVel(){
    return climberServo.getMaxVel();
  }
  public void setMaxVelocity(double vel){
    climberServo.setMaxVelocity(vel);
  }
  public double getCmdVelocity(){
    return cmdVel;
  }
  public boolean atSetpoint() {
    return Math.abs(getPosition() - getSetpoint()) < posTol;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.climberServo.periodic();
  }
class ClimberWatcherCmd extends WatcherCmd{ 
  NetworkTableEntry nt_cmdVelocity;
  NetworkTableEntry nt_measVelocity;
  NetworkTableEntry nt_measPosition;
  NetworkTableEntry nt_cmdPosition; //setpoint
  NetworkTableEntry nt_atSetpoint;
    // add nt for pos when we add it
  
    @Override
  public String getTableName() {
    return ClimberWatcherCmd.this.getName();
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
  nt_measPosition.setDouble(getPosition());
  nt_cmdPosition.setDouble(getSetpoint());
  nt_atSetpoint.setBoolean(atSetpoint());
}
}
}
