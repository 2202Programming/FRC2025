// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot2025.Constants.CAN;


public class Elevator_Subsystem extends SubsystemBase {
  /** Creates a new Elevator_Subsystem. */

  private enum Levels {
    LCoral, LOne, LTwo, LThree, LFour
  };
  private SparkMax Elevator_Motor = new SparkMax(CAN.Elevator, SparkMax.MotorType.kBrushless);

  public Elevator_Subsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   
  
  
}
