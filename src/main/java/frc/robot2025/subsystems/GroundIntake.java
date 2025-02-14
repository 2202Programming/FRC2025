// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.Constants.DigitalIO;

public class GroundIntake extends SubsystemBase {
  //TODO change degree values once we know actual positions. these are placeholders - er
  public enum Position {
    POWERUP(0.0, 0.0),      // pwr up could be different from ZERO
    ZERO(0.0, 0.0),
    ALGAE_PICKUP(45.0, 135.0),
    ALGAE_PLACE(20.0, 10.0), // algae place
    CORAL_PICKUP(100.0, 135.0), 
    CORAL_PLACE(35.0, 45.0), // coral place
    ALGAE_REST(35.0, 100.0),
    CORAL_REST(15.0, 45.0),
    FLOOR(135.0, 135.0);

    public double topval;
    public double btmval;

    private Position(double topval, double btmval) {
      this.topval = topval;
      this.btmval = btmval;
    }
  }
  // motor config constants
  final ClosedLoopSlot wheelSlot = ClosedLoopSlot.kSlot0;
  final int wheelStallLimit = 5;
  final int wheelFreeLimit = 5;
  final static double Kff = (1.0 / 43.2);
  final PIDFController wheelPIDF = new PIDFController(0.015, 0.0, 0.0, Kff); // TODO configure for velocity control. current vals are placeholders -er
  final static double wheelMtrGearRatio = 1.0 / 2.0; // 2 motor turns -> 1 wheel turn

// servo config
  final NeoServo topServo;
  final NeoServo btmServo;
  final SparkMax wheelMtr;
  DigitalInput lightgate = new DigitalInput(DigitalIO.GroundIntakeLightGate);
  final SparkMaxConfig wheelMtr_cfg;
  final SparkClosedLoopController wheelMtr_ctrl;
  final PIDController topPositionPID = new PIDController(0.75, 0.0, 0.0);
  final PIDController btmPositionPID = new PIDController(0.0, 0.0, 0.0);
  PIDFController topHwAngleVelPID = new PIDFController(0.0000, 0.0, 0.0, 0.00017); // placeholder PIDs
  PIDFController btmHwAngleVelPID = new PIDFController(0.0000, 0.0, 0.0, 0.0017);
  //TODO fix this its being devided by 60 in neoservo
  final double topServoGR = (1.0 /45.0) * 360.0 * 60; // 45:1 gearbox reduction * 360 degrees / turn 
  final double btmServoGR = (1.0 / 45.0) * 360.0 *60; // 45:1 gearbox reduction * 360 degrees / turn

  // Where we are heading, use atSetpoint to see if we are there
  Position currentPos = Position.POWERUP;

  public GroundIntake() {
    topServo = new NeoServo(CAN.IntakeTop, topPositionPID, topHwAngleVelPID, true);
    btmServo = new NeoServo(CAN.IntakeBtm, btmPositionPID, btmHwAngleVelPID, true);
    wheelMtr = new SparkMax(CAN.IntakeWheel, MotorType.kBrushless);
    topServo.setConversionFactor(topServoGR); // deg
    btmServo.setConversionFactor(btmServoGR); // deg

    // configure wheel motor
    
    wheelMtr_cfg = new SparkMaxConfig();
    wheelMtr_cfg.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(wheelStallLimit, wheelFreeLimit)
            .encoder   
              .positionConversionFactor(wheelMtrGearRatio) // rotations
              .velocityConversionFactor(wheelMtrGearRatio / 60.0); // rps

    wheelMtr_cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // finish pid and config 
    wheelPIDF.copyTo(wheelMtr, wheelMtr_cfg, wheelSlot); // velocity mode
    wheelMtr.configure(wheelMtr_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wheelMtr_ctrl = wheelMtr.getClosedLoopController();

    // Initialize our servo's power up conditions, does not change desired setpoint, 
    // it just sets the encoder values directly
    topServo.setPosition(currentPos.topval);
    btmServo.setPosition(currentPos.btmval);

    // set our requested setpoint with our public api POWERUP
    setPosition(currentPos); // changes setpoints
  }

  public void setPosition(Position cmd) {
    currentPos = cmd;
    topServo.setSetpoint(cmd.topval);
    btmServo.setSetpoint(cmd.btmval);
  }

  public void debugSetPosition(double top, double btm){
    topServo.setSetpoint(top);
    btmServo.setSetpoint(btm);
  }

  public void debugBtmVelocity(double dir){
    btmServo.setVelocityCmd(dir);
  }

  public void debugTopVelocity(double dir){
    topServo.setVelocityCmd(dir);
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

  public void setWheelSpeed(double speed){
    wheelMtr_ctrl.setReference(speed, ControlType.kVelocity);
  }

  public double getTopPosition(){
    return topServo.getPosition();
  }

  public double getBtmPosition(){
    return btmServo.getPosition();
  }

  public boolean senseGamePiece(){
    return !lightgate.get();
  }

  @Override
  public void periodic() {
    topServo.periodic();
    btmServo.periodic();
    // TODO read the lightgate
  }
}
