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
  final SparkClosedLoopController controller;
  final RelativeEncoder mtrEncoder;
  final SparkBaseConfig config;
  final double kF = 1.0 / 5500.0; // placeholder
  public final double adjustment = 0.0;
  private double cmdRPM;
  private double measRPM;

  //roller's pid values, will be copied to the hardware.
  PIDFController pid = new PIDFController(0.1, 0.0, 0.0, kF);
  final double velocityConversionFactor = (21.0 / 30.0) / 60.0; // GearRatio / sec -- RPS
  DigitalInput loadLightGate = new DigitalInput(DigitalIO.END_EFFECTOR_LOAD_LIGHTGATE);
  DigitalInput wheelLightGate = new DigitalInput(DigitalIO.END_EFFECTOR_WHEEL_LIGHTGATE);

  /** Creates a new EE_Subsystem. */
  public EndEffector_Subsystem() {
    mtr = new SparkMax(CAN.END_EFFECTOR, SparkMax.MotorType.kBrushless);    
    config = new SparkMaxConfig();

    // setup the mtr's config
    config.idleMode(IdleMode.kBrake)
      .inverted(true)   //note: motor may be inverted, but not encoder
      .encoder
        .positionConversionFactor(velocityConversionFactor * 60.0)  // [shaft-rot]
        .velocityConversionFactor(velocityConversionFactor);  // [shaft-rot/sec]
       
    //setup the config -- if closed loop is needed, use internal, primary encoder
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      //.iZone(150.0)  // placeholder
      //.iMaxAccum(150.0) // placeholder
      ;
    // writes pid's values and the above config setting to the hardware.
    // copyTo() will persist to hardware
    pid.copyTo(mtr, config);

    // get our controller and clear any faults
    controller = mtr.getClosedLoopController();
    mtrEncoder = mtr.getEncoder();
    mtr.clearFaults();
  }

  @Override
  public void periodic() {
    measRPM = mtrEncoder.getVelocity();
  }

  public boolean isAtRPM(double tolerance) {
    return (Math.abs(measRPM - cmdRPM) < tolerance);
  }

  public void setRPM(double RPM) {
    controller.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    cmdRPM = RPM;
  }

  public boolean hasPiece() {
    return loadLightGate.get();
  }

  public boolean pieceReady(){
    return wheelLightGate.get();
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