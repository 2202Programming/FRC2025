// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2025.Constants.AnalogIn;
import frc.robot2025.Constants.CAN;

public class WristFLA extends SubsystemBase {

  private static final double TOLERANCE = 0.04;
  private static final double k = 1.94123305;//.515;//1;// 2.0/5.0;

  private final SparkMax motor = new SparkMax(CAN.WRIST, MotorType.kBrushed);

  private final AnalogInput vPositionSensor = new AnalogInput(AnalogIn.Wrist);
  private final BangBangController bandBangController = new BangBangController(TOLERANCE);

  //final PIDController pid = new PIDController(kp, ki, kd);
  private double distance = 0.0;
  private double distanceCmd = 0.0;
  static double offset = .255126927;//.1313;//.255126927;//0.8305663212;

  double EtoETime = 3.0; //sec, time from 0 - 1.0 traveled
  double prevPos;
  double lastCommandTime;
  double timeToFinish;
  public static final double PICKUP_POSITION = 2; //pickup position from source
  public static final double MID_POSITION = 1.5; //drop position for L2/3
  public static final double Q3_POSITION = 0.4; //drop position for L2/3
  public static final double DROP_POSITION = 0; //drop position for L2/3

  public WristFLA() {
    SmartDashboard.putData(bandBangController);
    SparkBaseConfig driveCfg = new SparkMaxConfig().inverted(false).idleMode(IdleMode.kCoast); //TODO: coast??
    motor.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setPosition(2.5);
    System.out.println(vPositionSensor.getVoltage()*k);
    //pid.setTolerance(5,10);
  }

  public void setPosition(double position) {
    distanceCmd=position;
    bandBangController.setSetpoint(position);
}
  public void execute() {
    //timeToFinish = EtoETime * Math.abs(pos - prevPos) + Timer.getFPGATimestamp();
    //distance = vPositionSensor.getVoltage()*k - offset;

    distance = (vPositionSensor.getVoltage()-offset)*k;

    System.out.println(bandBangController.calculate(distance, distanceCmd) + "bbc.calculate(" + distance+ ", " + distanceCmd + " +)");

    if(!bandBangController.atSetpoint()) {

      if (distanceCmd >= distance) {
        //distance = vPositionSensor.getVoltage()*k + offset;
        //System.out.println(bandBangController.calculate(distance, distanceCmd) + "bbc.calculate(" + distance+ ", " + distanceCmd + " +)");
        motor.set(bandBangController.calculate(distance, distanceCmd));
      } else {
        //distance = vPositionSensor.getVoltage()*k - offset;
        //System.out.println(bandBangController.calculate(distanceCmd, distance) + "bbc.calculate(" + distanceCmd+ ", " + distance + " +)");
        motor.set(-1 * bandBangController.calculate(distanceCmd, distance));
      }
    }
    
  } 
  
  public void stop() {
    motor.set(0);
  }


  public double getEstimatedTime() {
    return timeToFinish;
  }

  public boolean atSetpoint() {
    return bandBangController.atSetpoint();
    //return Math.abs(distance - distanceCmd) < .01;
    //return Timer.getFPGATimestamp() >= timeToFinish;
  }
   
}
