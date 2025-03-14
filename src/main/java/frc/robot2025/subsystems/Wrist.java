// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  final static double PowerUpPosition = 0.0;  //TODO - confirm this.

  /** Creates a new linearServo. */
  PWM servo;

  double EtoETime = 3.0; //sec, time from 0 - 1.0 traveled
  double prevPos;
  double lastCommandTime;
  double timeToFinish;
  public final double pickup = 0.0;
  public final double drop = 0.5;

  public Wrist() {
    this(PowerUpPosition);
  }
  
  public Wrist(double initPos) {
    //works on a scale of 0-1, and autoclamps 
    servo = new PWM(0);
    prevPos = initPos;
    setPos(initPos);
  }

  public void setPos(double pos) {
    timeToFinish = EtoETime * Math.abs(pos - prevPos) + Timer.getFPGATimestamp();
    servo.setPosition(pos);
  } 
  

  public double getEstimatedTime() {
    return timeToFinish;
  }

  public boolean atSetpoint() {
    return Timer.getFPGATimestamp() >= timeToFinish;
  }
}
