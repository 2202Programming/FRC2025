// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearServo extends SubsystemBase {
  /** Creates a new linearServo. */
  PWM servo;

  public LinearServo() {
    //works on a scale of 0-1, and autoclamps 
    servo = new PWM(0);
    System.out.println(servo.getChannel());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPos(double pos) {
    servo.setPosition(pos);
  } 
  
  public double getPos() {
    return servo.getPosition();
  }
}
