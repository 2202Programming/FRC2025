// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

public class L4Coral extends SubsystemBase{
  /** Creates a new L4Coral. */
  NeoServo servo;
  PIDController servoPosPID;
  PIDFController hwVelPID;
  final int STALL_CURRENT = 40;
  final int FREE_CURRENT = 50;
  double currVel;
  final double maxVel = 5.0;
  double desiredVel;
  double accel;
  final double tolerance = 1.0;
  final double maxAccel = 5.0;
  final double gearRatio = 1.0 / 50.0;
  public L4Coral() {
    servoPosPID = new PIDController(0.0, 0.0, 0.0);
    hwVelPID = new PIDFController(0.0, 0.0, 0.0, 0.0);
    servo = new NeoServo(5, servoPosPID, hwVelPID, false);
    servo.setConversionFactor(gearRatio)
    .setTolerance(tolerance, tolerance)
    .setMaxVelocity(maxVel)
    .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
    .burnFlash();
  }

  @Override
  public void periodic() {
    servo.periodic();
  }

  public void setVelocity(double vel){
    servo.setVelocityCmd(vel);
  }
  public double getMaxVelocity(){
    return maxVel;
  }

  
}
