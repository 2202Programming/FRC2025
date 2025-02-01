// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib2202.util.NeoServo;
import frc.robot2025.Constants.CAN;


public class Elevator_Subsystem extends SubsystemBase {
  /** Creates a new Elevator_Subsystem. */

  private enum Levels {
    LCoral, LOne, LTwo, LThree, LFour //may add heights that corralate to these, double check with andrew on how to do
  };
  private final SparkMax Elevator_Motor;
  private final SparkMax Elevator_Motor2; 
  private SparkClosedLoopController elevatorPid;
  private final PIDController elevatorPidController;
  private NeoServo elevator_Servo;
  //what kind of sensors will we have on this bot. Not fully sure yet. Double check with mechanical
  private double elevator_velecity; // cm/sec
  private double elevator_height; //in cm
  private double elevator_height_setpoint; //in cm
  private SparkMaxConfig elevator_config;

  private int gear_Ratio = 5;  //throw in constants?
  private int chain_Ratio = 314159;
  

  public Elevator_Subsystem() {
    Elevator_Motor = new SparkMax(CAN.Elevator, SparkMax.MotorType.kBrushless);
    Elevator_Motor2 = new SparkMax(CAN.Elevator, SparkMax.MotorType.kBrushless);
    elevator_height = 0;
    elevator_config = new SparkMaxConfig();
    elevatorPidController = new PIDController(0, 0, 0);

    elevator_config
      .idleMode(IdleMode.kBrake);
    
    Elevator_Motor.configure(elevator_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Elevator_Motor2.configure(elevator_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   
  public double get_Elevator_Height() {
    return elevator_height; //some sort of calculation during the peridic, where we keep updating the positon of the elevator.
  }

  public void Elevator_Set_ToHeight(double height) {
    elevator_height_setpoint = height;
  }

  public void set_Elevator_Speed(double speed){ 
    this.elevator_velecity = speed;
  }

  public double get_elevator_setpoint() {
    return elevator_height_setpoint;
  }

  public double get_elevator_speed() {
    return elevator_velecity;
  }

  
}
