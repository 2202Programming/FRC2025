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
import frc.lib2202.command.WatcherCmd;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;


public class Elevator_Subsystem extends SubsystemBase {
  /** Creates a new Elevator_Subsystem. */

  /*
   * All placeholders 2/5/25
   *  maybe add algae level
   */
  public enum Levels {
    LCoral(75.5), 
    LOne(30), 
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
  private NeoServo elevatorServoMain; 
  private SparkMax elevatorServoFollow;
  private SparkMaxConfig followMotorConfig;
  private double desiredVel; //in cm/s

  final int STALL_CURRENT = 20;
  final int FREE_CURRENT = 40;
  final int elevatorMaxVel = 50; // [cm/s]
  final int elevatorMaxAccel = 40; // [cm/s]
  final int elevatorPosTol = 1;
  final int elevatorVelTol = 1;
  final int maxPos = 100;
  final int minPos = -10;

  private double gearRatio = 1.0/5.0;  //throw in constants?
  private int chainRatio = 314159; //TODO get valid number
  private int pullyRadius = 0; //TODO get valid number
  private int pullyStages = 3; //TODO get valid number
  private double conversionFactor = pullyRadius * gearRatio * Math.PI / 30.0;

  

  public Elevator_Subsystem() {
    desiredVel = 0;
    elevatorPidController = new PIDController(0, 0, 0);
    elevatorMechanicalPid = new PIDFController(0, 0, 0, 0);
    elevatorServoMain = new NeoServo(CAN.ELEVATOR_MAIN, elevatorPidController, elevatorMechanicalPid, false);
    elevatorServoFollow = new SparkMax(CAN.ELEVATOR_FOLLOW, MotorType.kBrushless); 
    
    //lines 51-55 config motor 2 the same as the first motor 
    elevatorServoMain.setConversionFactor(conversionFactor) //probably wrong, double check
                      .setTolerance(elevatorPosTol, elevatorPosTol)
                      .setVelocityHW_PID(elevatorMaxVel, elevatorMaxAccel)
                      .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
                      .burnFlash();
    elevatorServoMain.setClamp(minPos, maxPos);
    followMotorConfig = new SparkMaxConfig();
        followMotorConfig.inverted(false)
               .idleMode(IdleMode.kBrake);
    followMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
                .outputRange(-1.0, 1.0);
    
    elevatorServoFollow.configure(followMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followMotorConfig.follow(CAN.ELEVATOR_MAIN); //motor 2 follows the servo's behavior
  
  }

  @Override
  public void periodic() {
    elevatorServoMain.periodic();
  }
   
  public double getHeight() {
    return elevatorServoMain.getPosition();
  }

  public void setHeight (Levels level) {
    elevatorServoMain.setSetpoint(level.height); 
  }
  public void setHeight (double height) {
    elevatorServoMain.setSetpoint(height); 
  }

  public double getSetpoint() {
    return elevatorServoMain.getSetpoint();
  }

  public double getVelocity() {
    return elevatorServoMain.getVelocity();
  }

  public void setVelocity(double vel) {
    desiredVel = vel;
    elevatorServoMain.setVelocityCmd(vel);
  }

  public boolean atSetpoint() {
    return elevatorServoMain.atSetpoint();
  }

  public double getDesiredVelocity() {
    return desiredVel;
  }
public WatcherCmd getWatcher() {
    return new ElevatorWatcherCmd();
  }

   class ElevatorWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdVel;
    NetworkTableEntry nt_measVel;
    NetworkTableEntry nt_desiredHeight;
    NetworkTableEntry nt_currentHeight;
    NetworkTableEntry nt_atHeight;

    // add nt for pos when we add it
    @Override
    public String getTableName() {
      return Elevator_Subsystem.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdVel = table.getEntry("cmdVel");
      nt_measVel = table.getEntry("measVel");
      nt_desiredHeight = table.getEntry("desiredHeight");
      nt_currentHeight = table.getEntry("currentHeight");
      nt_atHeight = table.getEntry("atSetpoint");
    }

    public void ntupdate() {
      nt_cmdVel.setDouble(getDesiredVelocity());
      nt_measVel.setDouble(getVelocity());
      nt_desiredHeight.setDouble(getSetpoint());
      nt_currentHeight.setDouble(getHeight());
      nt_atHeight.setBoolean(atSetpoint());
    }
  }

  
}
