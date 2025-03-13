// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import java.time.OffsetDateTime;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.PIDFController;

public class WristFLA extends SubsystemBase {

  private static final double kp = 1.0;
  private static final double ki = 0.0;
  private static final double kd = 0.0;
  private static final double k = 6.0/5.0;

  final SparkMax motor = new SparkMax(26, MotorType.kBrushed);
  SparkBaseConfig driveCfg =  new SparkMaxConfig().inverted(false).idleMode(IdleMode.kCoast);

  final AnalogInput vPositionSensor = new AnalogInput(3);
  BangBangController bbc = new BangBangController();

  //final PIDController pid = new PIDController(kp, ki, kd);
  private double distance = 0.0;
  private double distanceCmd = 0.0;
  static double offset = 0.8305663212;

  double EtoETime = 3.0; //sec, time from 0 - 1.0 traveled
  double prevPos;
  double lastCommandTime;
  double timeToFinish;
  public static final double pickup = 6; //pickup position from source
  public final double drop = 0; //drop position for L2/3

  public WristFLA() {
    motor.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setPos(pickup);
    System.out.println(vPositionSensor.getVoltage()*k);
    //pid.setTolerance(5,10);
  }
  

  public void setPos(double pos) {
    //timeToFinish = EtoETime * Math.abs(pos - prevPos) + Timer.getFPGATimestamp();
    //distance = vPositionSensor.getVoltage()*k - offset;
    distanceCmd=pos;
    if(distanceCmd>=distance ) {
      distance = vPositionSensor.getVoltage()*k + offset;
      System.out.println(bbc.calculate(distance, distanceCmd) + "bbc.calculate(" + distance+ ", " + distanceCmd + " +)");
      motor.set(bbc.calculate(distance, distanceCmd));  
    } else {
      distance = vPositionSensor.getVoltage()*k - offset;
      System.out.println(bbc.calculate(distanceCmd, distance) + "bbc.calculate(" + distanceCmd+ ", " + distance + " +)");
      motor.set(-1*bbc.calculate(distanceCmd, distance));
    }
    
  } 
  
  public void stop() {
    motor.set(0);
  }


  public double getEstimatedTime() {
    return timeToFinish;
  }

  public boolean atSetpoint() {
    return Math.abs(distance - distanceCmd) < .01;
    //return Timer.getFPGATimestamp() >= timeToFinish;
  }
   
}
