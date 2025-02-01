// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;


public class Elevator_Subsystem extends SubsystemBase {
  /** Creates a new Elevator_Subsystem. */

  //get once we have prototype (USE ELLANA'S CODE AS EXAMPLE)
  public enum Levels {
    LCoral(75.5), 
    LOne(0), 
    LTwo(75.5), 
    LThree(116), 
    LFour(176),
    Ground(0); //change to accurate heights (in CM) THESE ARE NOT ACCURATE

    public double height;

    private Levels(double height) {
      this.height = height;
    }
  }; 

  private final PIDController elevatorPidController;
  private final PIDFController elevatorPIDF;
  private NeoServo elevator_Servo1;
  private SparkMax elevator_motor2;
  private SparkMaxConfig motor2_Config;
  //what kind of sensors will we have on this bot. Not fully sure yet. Double check with mechanical
  private double elevator_height; //in cm
  private double elevator_height_setpoint; //in cm

  private int gear_Ratio = 5;  //throw in constants?
  private int chain_Ratio = 314159; //TODO get valid number
  

  public Elevator_Subsystem() {
    elevatorPidController = new PIDController(0, 0, 0);
    elevatorPIDF = new PIDFController(0, 0, 0, 0);
    elevator_Servo1 = new NeoServo(CAN.Elevator ,elevatorPidController, elevatorPIDF, false);
    elevator_motor2 = new SparkMax(-10, MotorType.kBrushless); //TODO change CAN ID
    
    //lines 51-55 config motor 2 the same as the first motor 
    motor2_Config = new SparkMaxConfig();
        motor2_Config.inverted(false)
               .idleMode(IdleMode.kBrake);
    motor2_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
                .outputRange(-1.0, 1.0);
    
    elevator_motor2.configure(motor2_Config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor2_Config.follow(CAN.Elevator); //motor 2 follows the servo's behavior
    elevator_height = 0;
  
  }

  @Override
  public void periodic() {
    elevator_Servo1.periodic();
  }
   
  public double get_Elevator_Height() {
    return elevator_Servo1.getPosition(); //some sort of calculation needed
  }

  public void Elevator_Set_ToHeight(Levels level) {
    // double height = reverse of the equation from motor rotations to cm
    //elevator_height_setpoint = height;
    //elevator_Servo1.setSetpoint(height);  TODO this. In general. We could also throw the conversion in periodic
  }

  public double get_elevator_setpoint() {
    return elevator_height_setpoint;
  }

  public double get_elevator_speed() {
    return elevator_Servo1.getVelocity();
  }

  
}
