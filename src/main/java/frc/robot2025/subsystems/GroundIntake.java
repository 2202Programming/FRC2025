// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.Constants.DigitalIO;

public class GroundIntake extends SubsystemBase {
  // TODO change degree values once we know actual positions. these are
  // placeholders - er
  public enum Position {
    POWERUP(0.0, 0.0), // pwr up could be different from ZERO
    ZERO(0.0, 0.0),
    ALGAE_PICKUP(-45.0, 120.0),
    ALGAE_PLACE(-45.0, 60), // algae place
    ALGAE_REST(-45.0, 100.0),
    CORAL_PICKUP(-15.0, 120.0),
    CORAL_PLACE(-20.0, 45.0), // coral place
    CORAL_REST(-20.0, 45.0),
    FLOOR(0.0, 120.0);

    public double topval;
    public double btmval;

    private Position(double topval, double btmval) {
      this.topval = topval;
      this.btmval = btmval;
    }
  }

  // motor config constants
  final ClosedLoopSlot wheelSlot = ClosedLoopSlot.kSlot0;
  final int wheelStallLimit = 40;
  final int wheelFreeLimit = 5;
  final static double Kff = (1.0 / 43.2);
  final PIDFController wheelPIDF = new PIDFController(0.015, 0.0, 0.0, Kff); // TODO configure for velocity control.
                                                                             // current vals are placeholders -er
  final static double wheelMtrGearRatio = 1.0 / 2.0; // 2 motor turns -> 1 wheel turn

  final int StallCurrent = 40;
  final int FreeCurrent = 5;

  // servo config
  final NeoServo topServo;
  final NeoServo btmServo;
  final SparkMax wheelMtr;
  final RelativeEncoder wheelMtr_encoder;
  
  // gates for piece detection
  DigitalInput coralSwitch = new DigitalInput(DigitalIO.GroundIntakeHasCoral);
  DigitalInput algeSwitch = new DigitalInput(DigitalIO.GroundIntakeHasAlgae);

  final SparkMaxConfig wheelMtr_cfg;
  final SparkClosedLoopController wheelMtr_ctrl;
  public static final double WheelMaxVolts = 0.5;

  PIDFController topHwAngleVelPID = new PIDFController(0.00075, 0.0, 0.0, 0.0013); // placeholder PIDs
  final PIDController topPositionPID = new PIDController(3.5, 0.0, 0.0);

  PIDFController btmHwAngleVelPID = new PIDFController(0.0007, 0.000001, 0.0, 0.0017);
  final PIDController btmPositionPID = new PIDController(2.5, 0.0001, 0.0);

  final double topServoGR = (1.0 / 45.0) * 360.0; // 45:1 gearbox reduction * 360 degrees / turn
  final double btmServoGR = (1.0 / 45.0) * 360.0; // 45:1 gearbox reduction * 360 degrees / turn

  final double topIRange = 1.0; // degrees, default is infinity

  // Where we are heading, use atSetpoint to see if we are there
  Position currentPos = Position.POWERUP;
  double holdOffset;
  

  public GroundIntake() { 
    topHwAngleVelPID.setIZone(25.0);
    topPositionPID.setIntegratorRange(-topIRange, topIRange);
    btmHwAngleVelPID.setIZone(25.0);
    topServo = new NeoServo(CAN.IntakeTop, topPositionPID, topHwAngleVelPID, true);
    btmServo = new NeoServo(CAN.IntakeBtm, btmPositionPID, btmHwAngleVelPID, true);
    wheelMtr = new SparkMax(CAN.IntakeWheel, MotorType.kBrushless);
    topServo.setConversionFactor(topServoGR)
      .setMaxVelocity(90.0) // [deg/s]
      .setTolerance(4.0, 0.5)
      .setSmartCurrentLimit(StallCurrent, FreeCurrent);
    btmServo.setConversionFactor(btmServoGR)
      .setMaxVelocity(120.0)
      .setTolerance(2.0, 0.5)
      .setSmartCurrentLimit(StallCurrent, FreeCurrent);
  
   
    // configure wheel motor

    wheelMtr_cfg = new SparkMaxConfig();
    wheelMtr_cfg.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(wheelStallLimit, wheelFreeLimit).encoder
        .positionConversionFactor(wheelMtrGearRatio) // rotations
        .velocityConversionFactor(wheelMtrGearRatio / 60.0); // rps

    wheelMtr_cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // finish pid and config
    wheelPIDF.copyTo(wheelMtr, wheelMtr_cfg, wheelSlot); // velocity mode
    wheelMtr.configure(wheelMtr_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wheelMtr_ctrl = wheelMtr.getClosedLoopController();
    wheelMtr_encoder = wheelMtr.getEncoder();

    // Initialize our servo's power up conditions, does not change desired setpoint,
    // it just sets the encoder values directly
    topServo.setPosition(currentPos.topval);
    btmServo.setPosition(currentPos.btmval);

    // set our requested setpoint with our public api POWERUP
    setSetpoint(currentPos); // changes setpoints

    this.new GroundIntakeWatcher();
  }

  public void setSetpoint(Position cmd) {
    currentPos = cmd;
    setSetpoint(cmd.topval, cmd.btmval);
  }

  public void setSetpoint(double top, double btm) {
    topServo.setArbFeedforward(0.0);
    btmServo.setArbFeedforward(0.0);
    topServo.setSetpoint(top + holdOffset);
    btmServo.setSetpoint(btm);
  }

  public void debugBtmVelocity(double dir) {    
    double aff = 0.0;
    if(Math.abs(dir) > 0.5){
      aff = (dir > 0.0) ? 0.003 : -0.02;
    }
    if (dir== 0.0){
      aff = -0.02; // seemed to hold at 0 velocity
    }
    btmServo.setArbFeedforward(aff); 
    btmServo.setVelocityCmd(dir);
  }

  public void debugTopVelocity(double dir) {
    double aff = 0.0;
    if(Math.abs(dir) > 0.5){
      aff = (dir > 0.0) ? 0.003 : -0.02;
    }
    if (dir== 0.0){
      aff = -0.02;
    }
    topServo.setArbFeedforward(aff);
    topServo.setVelocityCmd(dir);
  }

  public void hold(double deg){
    this.holdOffset = deg;
  }

  public boolean isTopAtSetpoint() {
    return topServo.atSetpoint();
  }

  public boolean isBottomAtSetpoint() {
    return btmServo.atSetpoint();
  }

  public boolean isAtSetpoint() {
    return isTopAtSetpoint() && isBottomAtSetpoint();
  }

  public void setWheelSpeed(double speed) {
    wheelMtr_ctrl.setReference(speed, ControlType.kVelocity);
  }

  public void setWheelHold(double voltage){
    double clampVoltage = Math.abs(voltage) <= WheelMaxVolts ? voltage : WheelMaxVolts;
    clampVoltage = Math.copySign(clampVoltage, voltage);
    // MathUtil.clamp(voltage, -1.0 * WheelMaxVolts, WheelMaxVolts);
    // wheelMtr_ctrl.setReference(MathUtil.clamp(voltage, -1.0 * WheelMaxVolts, WheelMaxVolts);, ControlType.kVoltage);
    wheelMtr_ctrl.setReference(clampVoltage, ControlType.kVoltage);
  }
  public double getTopPosition() {
    return topServo.getPosition();
  }

  public double getBtmPosition() {
    return btmServo.getPosition();
  }

  public boolean senseCoral() {
    return !coralSwitch.get();
  }

  public boolean senseAlgae() {
    return !algeSwitch.get();
  }


  public void setZero(){
    topServo.setPosition(0.0);
    btmServo.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // protect against bad motion
    topServo.periodic();
    btmServo.periodic();
  }

  public class GroundIntakeWatcher extends WatcherCmd {
    // Table Entries odometry pose
    NetworkTableEntry NT_topVelocity;
    NetworkTableEntry NT_btmVelocity;
    NetworkTableEntry NT_wheelVelocity;
    NetworkTableEntry NT_hasCoral;
    NetworkTableEntry NT_hasAlgae;
    NetworkTableEntry NT_topPos;
    NetworkTableEntry NT_btmPos;
    NetworkTableEntry NT_topCmdVel;
    NetworkTableEntry NT_btmCmdVel;
    NetworkTableEntry NT_topCmdPos;
    NetworkTableEntry NT_btmCmdPos;
    NetworkTableEntry NT_topAtSetpoint;
    NetworkTableEntry NT_topGetIAccum;
    NetworkTableEntry NT_groundIntakeHasCoral;
    NetworkTableEntry NT_groundIntakeHasAlgae;

    public GroundIntakeWatcher() {
    }

    @Override
    public String getTableName() {
      return "Ground Intake";
    }

    @Override
    public void ntcreate() {
      NetworkTable MonitorTable = getTable();
      NT_topVelocity = MonitorTable.getEntry("top velocity");
      NT_btmVelocity = MonitorTable.getEntry("bottom velocity");
      NT_wheelVelocity = MonitorTable.getEntry("roller velocity");
      NT_hasCoral = MonitorTable.getEntry("hasCoral");
      NT_hasAlgae = MonitorTable.getEntry("hasAlgae");
      NT_topPos = MonitorTable.getEntry("top position");
      NT_btmPos = MonitorTable.getEntry("bottom position");
      NT_topCmdVel = MonitorTable.getEntry("top cmd velocity");
      NT_btmCmdVel = MonitorTable.getEntry("bottom cmd velocity");
      NT_topCmdPos = MonitorTable.getEntry("top cmd position");
      NT_btmCmdPos = MonitorTable.getEntry("bottom cmd position");
      NT_topAtSetpoint = MonitorTable.getEntry("is top at setpoint");
      NT_topGetIAccum = MonitorTable.getEntry("top IAccum");
    }

    @Override
    public void ntupdate() {
      NT_topVelocity.setDouble(topServo.getVelocity());
      NT_btmVelocity.setDouble(btmServo.getVelocity());
      NT_wheelVelocity.setDouble(wheelMtr_encoder.getVelocity());
      NT_hasCoral.setBoolean(senseCoral());
      NT_hasAlgae.setBoolean(senseAlgae());
      NT_topPos.setDouble(topServo.getPosition());
      NT_btmPos.setDouble(btmServo.getPosition());
      NT_topCmdVel.setDouble(topServo.getVelocityCmd());
      NT_btmCmdVel.setDouble(btmServo.getVelocityCmd());
      NT_topCmdPos.setDouble(topServo.getSetpoint());
      NT_btmCmdPos.setDouble(btmServo.getSetpoint());
      NT_topAtSetpoint.setBoolean(isTopAtSetpoint());
      NT_topGetIAccum.setDouble(topServo.getController().getClosedLoopController().getIAccum());

    }
  } 

}
