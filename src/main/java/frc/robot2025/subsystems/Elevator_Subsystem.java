// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;
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
  private NeoServo servo; 
  private SparkFlex followMotor;
  private SparkFlexConfig followMotorConfig;
  private double desiredVel; //in cm/s

  final int STALL_CURRENT = 20;
  final int FREE_CURRENT = 80;
  final double elevatorMaxVel = 5700.0; // [cm/s] rpm
  final double elevatorMaxAccel = 5000.0; // [cm/s^2]  servo may not enforce yet
  final double elevatorPosTol = 0.5;  // [cm]
  final double elevatorVelTol = 0.5;  // [cm]
  final double maxPos = 100.0; // [cm]
  final double minPos = 0.0;   // [cm]
  final double initPos = 0.0;  // [cm]  initial power up position for relative encoders
  final boolean motors_inverted = false;

  private final double gearRatio = 1/4.67; // [out turns]/[mtr turns]
  private final double chainRatio = 1.0;    // [out/in] chain in/out 
  private final double pullyRadius = 2.5;   // [cm]   TODO get valid number
  private final double stagesRatio = 1.0;   // [out/in]  TODO get valid number
  public  final double  pcf = gearRatio * stagesRatio * chainRatio * pullyRadius * 2.0 * Math.PI;
  private final double positionConversionFactor = 1.0;


  public Elevator_Subsystem() {
    desiredVel = 0;
    elevatorPidController = new PIDController(0.0, 0.0, 0.0);
    elevatorMechanicalPid = new PIDFController(0.000, 0.0, 0.0, 1.0/2700.0);
    servo = new NeoServo(CAN.ELEVATOR_MAIN, elevatorPidController, elevatorMechanicalPid, motors_inverted);
    followMotor = new SparkFlex(CAN.ELEVATOR_FOLLOW, MotorType.kBrushless); 
    
    //lines 51-55 config motor 2 the same as the first motor 
    servo.setConversionFactor(positionConversionFactor) //probably wrong, double check
                      .setTolerance(elevatorPosTol, elevatorPosTol)
                      .setVelocityHW_PID(elevatorMaxVel, elevatorMaxAccel)
                      .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);

    servo.setClamp(minPos, maxPos);
    followMotorConfig = new SparkFlexConfig();
        followMotorConfig.inverted(motors_inverted)
               .idleMode(IdleMode.kBrake);
    followMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
                .outputRange(-1.0, 1.0);
    
    followMotorConfig.follow(CAN.ELEVATOR_MAIN); //motor 2 follows the servo's behavior
    followMotor.configure(followMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // power up config
    servo.setPosition(initPos);
  
  }

  @Override
  public void periodic() {
    servo.periodic();
  }
   
  public double getHeight() {
    return servo.getPosition();
  }

  // current access doesn't need to be exposed, putting on NT
  double getMainCurrent(){
    return servo.getController().getOutputCurrent();
  }

  double getFollowCurrent() {
    return followMotor.getOutputCurrent();
  }

  public void setHeight (Levels level) {
    setHeight(level.height); 
  }

  public void setHeight (double height) {
    servo.setSetpoint(height); 
  }

  public double getSetpoint() {
    return servo.getSetpoint();
  }

  public double getVelocity() {
    return servo.getVelocity();
  }

  public void setVelocity(double vel) {
    desiredVel = vel;
    servo.setVelocityCmd(vel);
  }

  public boolean atSetpoint() {
    return servo.atSetpoint();
  }

  public double getDesiredVelocity() {
    return desiredVel;
  }

  public WatcherCmd getWatcher() {
    return this.new ElevatorWatcherCmd();
  }

   class ElevatorWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdVel;
    NetworkTableEntry nt_measVel;
    NetworkTableEntry nt_desiredHeight;
    NetworkTableEntry nt_currentHeight;
    NetworkTableEntry nt_atHeight;
    NetworkTableEntry nt_mainCurrent;
    NetworkTableEntry nt_followCurrent;

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
      nt_mainCurrent = table.getEntry("mainCurrent");
      nt_followCurrent = table.getEntry("followCurrent");
    }

    public void ntupdate() {
      nt_cmdVel.setDouble(getDesiredVelocity());
      nt_measVel.setDouble(getVelocity());
      nt_desiredHeight.setDouble(getSetpoint());
      nt_currentHeight.setDouble(getHeight());
      nt_atHeight.setBoolean(atSetpoint());
      nt_mainCurrent.setDouble(getMainCurrent());
      nt_followCurrent.setDouble(getFollowCurrent());
    }
  }

  
}
