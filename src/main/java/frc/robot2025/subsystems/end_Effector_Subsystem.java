// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class end_Effector_Subsystem extends SubsystemBase {
  /** Creates a new end_Effector_Subsystem. */
  private DoubleSolenoid EE_Solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0); //change line depending on module type. Also, CAN bus stuff
  private SparkMax end_effector_motor = new SparkMax(0, MotorType.kBrushless); //TODO: can ID's
  //depending on how we use the end effector, we may want a pid to control velocity. On the other hand, we could just comand power to 100%
  private double end_Effector_Speed;

  public end_Effector_Subsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set_Speed(double speed) {
      end_Effector_Speed = speed;
  }

  public double get_Speed() {
    return end_Effector_Speed;
  }

  public boolean solenoid_Pos_Up() {
    return EE_Solenoid.isRevSolenoidDisabled(); //may change depending on design
  }

  public void switch_Position() {
    if (solenoid_Pos_Up()) {
      EE_Solenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      EE_Solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }


}
