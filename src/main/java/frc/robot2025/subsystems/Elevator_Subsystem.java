// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot2025.Constants.CAN;


public class Elevator_Subsystem extends SubsystemBase {
  /** Creates a new Elevator_Subsystem. */

  private enum Levels {
    LCoral, LOne, LTwo, LThree, LFour //may add heights that corralate to these, double check with andrew on how to do
  };
  private SparkMax Elevator_Motor = new SparkMax(CAN.Elevator, SparkMax.MotorType.kBrushless);
  private SparkMax Elevator_Motor2 = new SparkMax(CAN.Elevator2, SparkMax.MotorType.kBrushless); 
  //what kind of sensors will we have on this bot. Not fully sure yet. Double check with mechanical
  private double elevator_speed; //what will "speed" be in? I'm thinking inches per second right now \*_*/
  private double elevator_height; //in inches
  

  public Elevator_Subsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   
  public double get_Elevator_Height() {
    return 0.0; //some sort of calculation, or durring the peridic we keep updating the positon of the elevator.
  }

  public void Elevator_Set_ToHeight(double height) {

  }

  public void set_Elevator_Speed(double speed){ 
    this.elevator_speed = speed;
  }

  
}
