// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.Constants.DigitalIO;

public class EndEffector_Subsystem extends SubsystemBase {
  final SparkMax mtr;
  private double cmdRPM;
  private double measRPM;

  DigitalInput loadLightGate = new DigitalInput(DigitalIO.END_EFFECTOR_LOAD_HIGH_LIGHTGATE);  // false is broken(coral loaded), true is not broken(no coral)
  DigitalInput wheelLightGate = new DigitalInput(DigitalIO.END_EFFECTOR_WHEEL_LOW_LIGHTGATE);

  /** Creates a new EE_Subsystem. */
  public EndEffector_Subsystem() {
    mtr = new SparkMax(CAN.END_EFFECTOR, SparkMax.MotorType.kBrushless); //26 on bot on board 3
    mtr.clearFaults();
  }

  @Override
  public void periodic() {
    measRPM = mtr.getEncoder().getVelocity();
  }

  public boolean isAtRPM(double tolerance) {
    return (Math.abs(measRPM - cmdRPM) < tolerance);
  }

  public void setRPM(double RPM) {
    mtr.set(RPM);
    cmdRPM = RPM;
  }

  public boolean hasPiece() {
    return !loadLightGate.get();
  }

  public boolean pieceReady(){
    return !wheelLightGate.get();
  }
  
  public WatcherCmd getWatcher() {
    return this.new EndEffectorWatcherCmd();
  }

  class EndEffectorWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdRPM;
    NetworkTableEntry nt_measRPM;
    NetworkTableEntry nt_hasPiece;
    NetworkTableEntry nt_pieceReady;

    // add nt for pos when we add it
    @Override
    public String getTableName() {
      return EndEffector_Subsystem.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdRPM = table.getEntry("cmdRPM");
      nt_measRPM = table.getEntry("measRPM");
      nt_hasPiece = table.getEntry("hasPiece");
      nt_pieceReady = table.getEntry("pieceReady");
    }

    public void ntupdate() {
      nt_cmdRPM.setDouble(cmdRPM);
      nt_measRPM.setDouble(measRPM);
      nt_hasPiece.setBoolean(hasPiece());
      nt_pieceReady.setBoolean(pieceReady());
    }
  }

}