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
  final SparkClosedLoopController controller;
  final RelativeEncoder encoder;
  SparkBaseConfig config;
  final double kF = 1.0 / 5500.0; // placeholder
  public final double adjustment = 0.0;
  private double cmdRPM;
  private double measRPM;
  PIDFController pid = new PIDFController(0.1, 0.0, 0.0, kF);
  PIDFController pidConsts_freeSpin = new PIDFController(0.0, 0.0, 0.0, 0.0);
  final double velocityConversionFactor = 1.0;
  DigitalInput farLightGate = new DigitalInput(DigitalIO.EndEffector_nearLightgate);
  DigitalInput nearLightGate = new DigitalInput(DigitalIO.EndEffector_farLightgate);

  /** Creates a new EE_Subsystem. */
  public EndEffector_Subsystem() {
    mtr = new SparkMax(CAN.END_EFFECTOR, SparkMax.MotorType.kBrushless);
    encoder = mtr.getEncoder();
    controller = motor_config(mtr, pid, false);

  }

  @Override
  public void periodic() {
    measRPM = encoder.getVelocity();
  }

  public boolean isAtRPM(double tolerance) {
    return (Math.abs(measRPM - cmdRPM) < tolerance);
  }

  public void setRPM(double RPM) {
    if (RPM == 0.0) {
      controller.setIAccum(0.0);
      controller.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }
    controller.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    cmdRPM = RPM;
  }

  public boolean hasPiece() {
    return farLightGate.get();
  }

  public boolean pieceReady(){
    return nearLightGate.get();
  }

  // configure our motor controller and retun it
  SparkClosedLoopController motor_config(SparkMax mtr, PIDFController hwPidConsts, boolean inverted) {
    mtr.clearFaults();
    config.encoder.velocityConversionFactor(velocityConversionFactor); // idk if this is the best way to do it?
    // mtr.restoreFactoryDefaults(); //removed from API, shouldn't need
    var mtrpid = mtr.getClosedLoopController();
    pid.copyTo(mtr, config);
    config.closedLoop.iZone(150.0); // placeholder
    pidConsts_freeSpin.copyTo(mtr, config);
    config.closedLoop.iMaxAccum(150.0);
    // mtr.setInverted(inverted); //deprecated
    config.idleMode(IdleMode.kBrake);
    return mtrpid;
  }

  public WatcherCmd getWatcher() {
    return this.new EndEffectorWatcherCmd();
  }

  class EndEffectorWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdRPM;
    NetworkTableEntry nt_measRPM;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kF;
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
      nt_kP = table.getEntry("kP");
      nt_kF = table.getEntry("kF");
      nt_hasPiece = table.getEntry("hasPiece");
      nt_pieceReady = table.getEntry("pieceReady");
    }

    public void ntupdate() {
      nt_cmdRPM.setDouble(cmdRPM);
      nt_measRPM.setDouble(measRPM);
      nt_kP.setDouble(pid.getP());
      nt_kF.setDouble(pid.getF());
      nt_hasPiece.setBoolean(hasPiece());
      nt_pieceReady.setBoolean(pieceReady());
    }
  }

}