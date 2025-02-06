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
  private final PIDFController elevatorMechanicalPid;
  private NeoServo elevator_Servo1;
  private SparkMax elevator_motor2;
  private SparkMaxConfig motor2_Config;
  //what kind of sensors will we have on this bot. Not fully sure yet. Double check with mechanical
  private double elevator_height; //in cm
  private double elevator_height_setpoint; //in cm

  private int gear_Ratio = 5;  //throw in constants?
  private int chain_Ratio = 314159; //TODO get valid number
  private int pully_Radius = 0; //TODO get valid number
  private int pully_Stages = 3; //TODO get valid number
  

  public Elevator_Subsystem() {
    elevatorPidController = new PIDController(0, 0, 0);
    elevatorMechanicalPid = new PIDFController(0, 0, 0, 0);
    elevator_Servo1 = new NeoServo(CAN.Elevator ,elevatorPidController, elevatorMechanicalPid, false);
    elevator_motor2 = new SparkMax(-10, MotorType.kBrushless); //TODO change CAN ID
    
    //lines 51-55 config motor 2 the same as the first motor 
    elevator_Servo1.setConversionFactor((1/gear_Ratio)*pully_Radius*pully_Stages*chain_Ratio); //probably wrong, double check
    elevator_Servo1.setTolerance(0.5, 1.0);
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
    elevator_height = get_Height(); //maybe not needed, maybe throw in network tables
  }
   
  public double get_Height() {
    return 0.0; //(elevator_Servo1.getPosition() / gear_Ratio) * chain_Ratio * Math.PI * 2 * pully_Radius * pully_Stages; //TODO fix. Please
  }

  public void set_toHeight(Levels level) {
    elevator_Servo1.setSetpoint(level.height); 
  }
  public void set_toHeight(double height) {
    elevator_Servo1.setSetpoint(height); 
  }

  public double get_setpoint() {
    return elevator_height_setpoint;
  }

  public double get_speed() {
    return elevator_Servo1.getVelocity();
  }

  public void set_speed(double speed) {
    elevator_Servo1.setVelocityCmd(1.0);
  }

  public boolean at_setpoint() {
    return elevator_Servo1.atSetpoint();
  }

  
}
