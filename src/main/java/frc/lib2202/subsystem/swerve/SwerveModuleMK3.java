package frc.lib2202.subsystem.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.util.ModMath;
import frc.lib2202.util.PIDFController;

import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;

//import frc.robot2024.Constants.CAN;

public class SwerveModuleMK3 {
  public final String NT_Name = "DT";

  // PID slot for angle and drive pid on SmartMax controller
  final int kSlot = 0;

  private int frameCounter = 0;

  // Chassis config used for geometry and pathing math
  private final ChassisConfig cc;

  // Rev devices
  private final SparkMax driveMotor;
  private final SparkMax angleMotor;
  private final SparkClosedLoopController driveMotorPID;
  private final SparkClosedLoopController angleMotorPID; // sparkmax PID can only use internal NEO encoders
  private final RelativeEncoder angleEncoder; // aka internalAngle
  private final RelativeEncoder driveEncoder;
  
  private final SparkMaxConfig driveConfig;
  private final SparkMaxConfig angleConfig;

  // CTRE devices
  private final CANcoder absEncoder; // aka externalAngle (external to Neo/Smartmax)
  private double angleCmdInvert;

  /**
   * Warning CANCoder and CANEncoder are very close in name but very different.
   * 
   * CANCoder: CTRE, absolute position mode, +/- 180 CCW= positive CANEncoder:
   * RevRobotics, relative position only, must configure to CCW based on side &
   * gearing Continous positon so postion can be greater than 180 because it can
   * "infinitely" rotate. Cannot be inverted in Brushless mode, must invert motor
   * 
   */

  // NetworkTables
  String NTPrefix;

  // m_ -> measurements made every period - public so they can be pulled for
  // network tables...
  double m_internalAngle; // measured Neo unbounded [deg]
  double m_externalAngle; // measured CANCoder bounded +/-180 [deg]
  double m_velocity; // measured velocity [wheel's-units/s] [m/s]
  double m_position; // measure wheel positon for calibraiton [m]
  double m_angle_target; // desired angle unbounded [deg]
  double m_vel_target; // desired velocity [wheel's-units/s] [m/s]
  /**
   * SwerveModuleMK3 -
   * 
   * SmartMax controllers used for angle and velocity motors.
   * 
   * SmartMax Velocity mode is used to close the velocity loop. Units will match
   * the units of the drive-wheel-diameter.
   * 
   * Angle in degrees is controlled using position mode on the SmartMax. The angle
   * positon is not constrainted to +/- 180 degrees because the Neo has 32bit
   * float resolution, so we can just let the postion grow or shrink based on the
   * how many degrees we need to change. We could rotate 1000's of time without
   * going past the resolution of the SmartMax's position tracking. [deg]
   * 
   * Example: cmd_angle = 175 ==> 175 + (n * 360) where -Turns < n < Turns ==> ...
   * -545 == -185 == 175 == 535 == 895 ...
   * 
   * Minimum number of turns in one direction before we would have to consider
   * overflow: Turns = posBitResolution / encoder-counts Turns = 2^23 / (42*12.8)
   * = 15,603
   * 
   * Batteries will need changing before then.
   * 
   */
  public String myprefix;
  private CANcoderConfiguration absEncoderConfiguration;

  public SwerveModuleMK3(SparkMax driveMtr, SparkMax angleMtr, CANcoder absEnc,
      boolean invertAngleMtr, boolean invertAngleCmd, boolean invertDrive, String prefix) {
    driveMotor = driveMtr;
    angleMotor = angleMtr;
    absEncoder = absEnc;
    myprefix = prefix;

    IRobotSpec specs = RobotContainer.getRobotSpecs();
    RobotLimits limits = specs.getRobotLimits();

    // cc is the chassis config for all our pathing math
    cc = specs.getChassisConfig();

    driveConfig = new SparkMaxConfig();
    angleConfig = new SparkMaxConfig();

    driveConfig
        .inverted(invertDrive)
        .smartCurrentLimit(limits.driveStallAmp, limits.freeAmp)
        .idleMode(IdleMode.kBrake);
    driveConfig.encoder     // set driveEncoder to use units of the wheelDiameter, meters
        .positionConversionFactor(Math.PI * cc.wheelDiameter / cc.kDriveGR) // mo-rot to wheel units
        .velocityConversionFactor((Math.PI * cc.wheelDiameter / cc.kDriveGR) / 60.0); // mo-rpm wheel units
    driveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(cc.drivePIDF.getP(), cc.drivePIDF.getI(), cc.drivePIDF.getD(), cc.drivePIDF.getF());

    angleConfig
        .inverted(invertAngleMtr)
        .smartCurrentLimit(limits.angleStallAmp, limits.freeAmp)
        .idleMode(IdleMode.kBrake);
    angleConfig.encoder // set angle endcoder to return values in deg and deg/s
        .positionConversionFactor(360.0 / cc.kSteeringGR) // mo-rotations to degrees
        .velocityConversionFactor(360.0 / cc.kSteeringGR / 60.0); // rpm to deg/s
    angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(cc.anglePIDF.getP(), cc.anglePIDF.getI(), cc.anglePIDF.getD(), cc.anglePIDF.getF());

    REVLibError driveError = driveMtr.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    REVLibError angleError = angleMtr.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if(driveError != REVLibError.kOk)
      System.out.println("*** ERROR *** " + prefix + " Drive Motor Flash Failed. Error val=" + driveError);
    if(angleError != REVLibError.kOk)
      System.out.println("*** ERROR *** " + prefix + " Angle Motor Flash Failed. Error val=" + angleError);     

   // account for command sign differences if needed
    angleCmdInvert = (invertAngleCmd) ? -1.0 : 1.0;
    //dpl removed, set in parent setMagOffset(offsetDegrees);

    // Drive Motor config

    driveMotorPID = driveMotor.getClosedLoopController();
    driveEncoder = driveMotor.getEncoder();

    // Angle Motor config

    angleMotorPID = angleMotor.getClosedLoopController();
    angleEncoder = angleMotor.getEncoder();

//Probably not needed? We check once.
    // // burn the motor flash if BURN_FLASH is true in frc.robot.Constants.CAN
    // if (specs.burnFlash()) {
    //   REVLibError angleError = angleMotor.burnFlash();
    //   sleep(1500); // takes 1 sec to burn per Dean

    //   int counter = 0;
    //   while (angleError.value != 0) {
    //     System.out.println(prefix + " angle error: " + angleError.value);
    //     counter++;
    //     if (counter > 20) {
    //       System.out.println("*** ERROR *** " + prefix + " Angle Motor Flash Failed.");
    //       break;
    //     }
    //     sleep(100);
    //   }
    //   System.out.println(myprefix + " Angle motor flash success.");

    //   REVLibError driveError = driveMotor.burnFlash();
    //   sleep(1500); // takes 1 sec to burn per Dean
    //   counter = 0;
    //   while (driveError.value != 0) {
    //     System.out.println(prefix + " drive error: " + driveError.value);
    //     counter++;
    //     if (counter > 20) {
    //       System.out.println("*** ERROR *** " + prefix + " Drive Motor Flash Failed.");
    //       break;
    //     }
    //     sleep(100);
    //   }
    //   System.out.println(myprefix + " Drive motor flash success.");
    // } else {
    //   System.out.println("Skipped burning flash.");
    // }
    /*
     * setNTPrefix - causes the network table entries to be created and updated on
     * the periodic() call.
     * 
     * Use a short string to indicate which MK unit this is.
     */
    NTPrefix = "/MK3-" + prefix;
    myprefix = prefix;
    NTConfig();

    calibrate();

  }

  // PID accessor for use in Test/Tune Commands
  public void setDrivePID(PIDFController temp) {
    temp.copyTo(driveMotorPID, kSlot);
  }

  public void setAnglePID(PIDFController temp) {
    temp.copyTo(angleMotorPID, kSlot);
  }

  /**
   * This adjusts the absEncoder with the given offset to correct for CANCoder
   * mounting position. This value should be persistent accross power cycles.
   * 
   * Warning, we had to sleep afer setting configs before the absolute position
   * could be read in calibrate.
   * 
   * Deprecated, cancoder fully calibrated in SwerveDrivetrain initCANcoder() now. 8/18/24
   * 
   * @param offsetDegrees
   */
  @Deprecated
  void setMagOffset(double offsetDegrees) {
    // adjust magnetic offset in absEncoder, measured constants.
    absEncoderConfiguration = new CANcoderConfiguration();
    absEncoderConfiguration.withMagnetSensor(new MagnetSensorConfigs());
    absEncoderConfiguration.MagnetSensor.MagnetOffset = offsetDegrees / 360.0;
    absEncoder.getConfigurator().apply(absEncoderConfiguration);

    System.out.println("Module " + myprefix + ": Set Offset=" + offsetDegrees + ". Initial CANCODER absolute angle="
        + absEncoder.getAbsolutePosition() + ", Initial CANCODER angle=" + absEncoder.getPosition());

    // if different, update
    //if (offsetDegrees != absEncoderConfiguration.MagnetSensor.MagnetOffset) {
    //  absEncoderConfiguration.MagnetSensor.MagnetOffset = offsetDegrees;
    //  absEncoder.getConfigurator().apply(absEncoderConfiguration);
    //}

    System.out.println("Module " + myprefix + ": CANCODER Angle after offset programmed=" + absEncoder.getPosition());

  }

  /**
   * calibrate() - aligns Neo internal position with absolute encoder. This needs
   * to be done at power up, or when the unbounded encoder gets close to its
   * overflow point.
   */
  void calibrate() {
    // read absEncoder position, set internal angleEncoder to that value adjust for
    // cmd inversion.
    // Average a couple of samples of the absolute encoder
    double pos_deg = absEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    //sleep(10);
    //double absPosition = absEncoder.getAbsolutePosition().getValue() * 360.0;
    //pos_deg = (pos_deg + absPosition) / 2.0;

    angleEncoder.setPosition(angleCmdInvert * pos_deg);
    sleep(100); // sparkmax gremlins
    double temp = angleEncoder.getPosition();
    sleep(100); // sparkmax gremlins

    int counter = 0;
    while (Math.abs(pos_deg - temp) > 0.1) { // keep trying to set encoder angle if it's not matching
      angleEncoder.setPosition(angleCmdInvert * pos_deg);
      sleep(100); // sparkmax gremlins
      temp = angleEncoder.getPosition();
      sleep(100); // sparkmax gremlins
      if (counter++ > 20) {
        System.out.println("*** Angle position set failed after 20 tries ***");
        break;
      }
    }

    realityCheckSparkMax(angleCmdInvert * pos_deg, temp);

    System.out.println("Module " + myprefix + ": NEO post-calibrate angle=" + angleEncoder.getPosition());

  }

  void realityCheckSparkMax(double angle_cancoder, double internal_angle) {
    boolean result = true;

    double driveReportedConversionFactor = driveMotor.configAccessor.encoder.getPositionConversionFactor();
    if (Math.abs(driveReportedConversionFactor - Math.PI * cc.wheelDiameter / cc.kDriveGR) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " position conversion factor incorrect for drive");
      System.out.println("Expected Position CF: " + Math.PI * cc.wheelDiameter / cc.kDriveGR);
      System.out.println("Returned Position CF: " + driveReportedConversionFactor);
      result = false;
    }

    double driveReportedVelocityFactor = driveMotor.configAccessor.encoder.getVelocityConversionFactor();
    if (Math.abs(driveReportedVelocityFactor
        - Math.PI * cc.wheelDiameter / cc.kDriveGR / 60.0) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " velocity conversion factor incorrect for drive");
      System.out.println("Expected Vel CF: " + Math.PI * cc.wheelDiameter / cc.kDriveGR / 60.0);
      System.out.println("Returned Vel CF: " + driveReportedVelocityFactor);
      result = false;
    }

    double angleReportedPositionConversionFactor = angleMotor.configAccessor.encoder.getPositionConversionFactor();
    if (Math.abs(angleReportedPositionConversionFactor - (360.0 / cc.kSteeringGR)) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " position conversion factor incorrect for angle");
      System.out.println("Expected Angle Pos CF: " + 360.0 / cc.kSteeringGR);
      System.out.println("Returned Angle Pos CF: " + angleReportedPositionConversionFactor);
      result = false;
    }

    double angleReportedVelocityConversionFactor = angleMotor.configAccessor.encoder.getVelocityConversionFactor();
    if (Math.abs(angleReportedVelocityConversionFactor - (360.0 / cc.kSteeringGR / 60)) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " velocity conversion factor incorrect for angle");
      System.out.println("Expected Angle Vel CF: " + (360.0 / cc.kSteeringGR / 60));
      System.out.println("Returned Angle Vel CF: " + angleReportedVelocityConversionFactor);
      result = false;
    }

    if (Math.abs(angle_cancoder - internal_angle) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " angle encoder save error");
      System.out.println("Expected internal angle: " + angle_cancoder);
      System.out.println("Returned internal angle: " + internal_angle);
      result = false;
    }
    if (result) {
      System.out.println(myprefix + " passed reality checks.");
    }
    return;
  }

  // _set<> for testing during bring up.
  public void _setInvertAngleCmd(boolean invert) {
    angleCmdInvert = (invert) ? -1.0 : 1.0;
    calibrate();
  }

  public void _setInvertAngleMotor(boolean invert) {
    angleMotor.setInverted(invert);
  }

  public void _setInvertDriveMotor(boolean invert) {
    driveMotor.setInverted(invert);
  }

  /**
   * setNTPrefix - causes the network table entries to be created and updated on
   * the periodic() call.
   * 
   * Use a short string to indicate which MK unit this is.
   * 
   *
   * public SwerveModuleMK3 setNTPrefix(String prefix) { NTPrefix = "/MK3-" +
   * prefix; myprefix = prefix; NTConfig(); return this; }
   */

  public String getNTPrefix() {
    return NTPrefix;
  }

  public void periodic() {
    // measure everything at same time; these get updated every cycle
    m_internalAngle = angleEncoder.getPosition() * angleCmdInvert;
    m_velocity = driveEncoder.getVelocity();
    m_position = driveEncoder.getPosition();
    m_externalAngle = absEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    // these are for human consumption, update slower
    if (frameCounter++ > 10) {      
      NTUpdate();
      frameCounter = 0;
    }
  }

  public void simulationPeriodic() {

  }

  /**
   * This is the angle being controlled, so it should be thought of as the real
   * angle of the wheel.
   * 
   * @return SmartMax/Neo internal angle in Rotation2d object [rad]
   */
  public Rotation2d getAngleRot2d() {
    return Rotation2d.fromDegrees(m_internalAngle);
  }

  public double getAngle() {
    return m_internalAngle;
  }

  /**
   * External Angle is external to the SmartMax/Neo and is the absolute angle
   * encoder.
   * 
   * At power-up, this angle is used to calibrate the SmartMax PID controller.
   * 
   */
  public Rotation2d getAngleExternalRot2d() {
    return Rotation2d.fromDegrees(m_externalAngle);
  }

  public double getAngleExternal() {
    return m_externalAngle;
  }

  /**
   * 
   * @return velocity wheel's units [m]
   */
  public double getVelocity() {
    return m_velocity;
  }

  /**
   * 
   * @return velocity wheel's units [m]
   */
  public double getPosition() {
    return m_position;
  }

  // Expose the position with wpi class, [m], [rad]
  public SwerveModulePosition getSMPosition() {
    return new SwerveModulePosition(m_position, getAngleRot2d());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * 
   * @param desiredState - A SwerveModuleState representing the desired new state
   *                     of the module
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState m_state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(m_internalAngle)); // should
                                                                                                            // favor
                                                                                                            // reversing
                                                                                                            // direction
                                                                                                            // over
    // turning > 90 degrees
    state = m_state; // uncomment to use optimized angle command
    // use position control on angle with INTERNAL encoder, scaled internally for
    // degrees
    m_angle_target = m_state.angle.getDegrees();

    // figure out how far we need to move, target - current, bounded +/-180
    double delta = ModMath.delta360(m_angle_target, m_internalAngle);
    // if we aren't moving, keep the wheels pointed where they are
    if (Math.abs(m_state.speedMetersPerSecond) < .01)
      delta = 0;

    // now add that delta to unbounded Neo angle, m_internal isn't range bound
    angleMotorPID.setReference(angleCmdInvert * (m_internalAngle + delta), ControlType.kPosition);

    //save target vel for plots
    m_vel_target =m_state.speedMetersPerSecond;
    
    // use velocity control
    driveMotorPID.setReference(m_state.speedMetersPerSecond, ControlType.kVelocity);
  }

  /**
   * Network Tables data
   * 
   * If a prefix is given for the module, NT entries will be created and updated
   * on the periodic() call.
   * 
   */
  private NetworkTable table;
  private NetworkTableEntry nte_angle;
  private NetworkTableEntry nte_external_angle;
  private NetworkTableEntry nte_velocity;
  private NetworkTableEntry nte_position;
  private NetworkTableEntry nte_angle_target;
  private NetworkTableEntry nte_vel_target;
  private NetworkTableEntry nte_motor_current;
  private NetworkTableEntry nte_applied_output;

  void NTConfig() {
    // direct networktables logging
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nte_angle = table.getEntry(NTPrefix + "/angle");
    nte_external_angle = table.getEntry(NTPrefix + "/angle_ext");
    nte_velocity = table.getEntry(NTPrefix + "/velocity");
    nte_angle_target = table.getEntry(NTPrefix + "/angle_target");
    nte_vel_target = table.getEntry(NTPrefix + "/velocity_target");
    nte_position = table.getEntry(NTPrefix + "/position");
    nte_motor_current = table.getEntry(NTPrefix + "/motor_current");
    nte_applied_output = table.getEntry(NTPrefix + "/applied_output");
  }

  void NTUpdate() {
    if (table == null)
      return; // not initialized, punt
    nte_angle.setDouble(m_internalAngle);
    nte_external_angle.setDouble(m_externalAngle);
    nte_velocity.setDouble(m_velocity);
    nte_position.setDouble(m_position);
    nte_angle_target.setDouble(m_angle_target);
    nte_vel_target.setDouble(m_vel_target);
    nte_motor_current.setDouble(driveMotor.getOutputCurrent());
    nte_applied_output.setDouble(driveMotor.getAppliedOutput());
  }

  public static void sleep(long ms) {
    try {
      Thread.sleep(ms);
    } catch (Exception e) {
    }
  }

  SparkClosedLoopController getDrivePID() {
    return driveMotorPID;
  }

  SparkClosedLoopController getAnglePID() {
    return angleMotorPID;
  }

  public void setBrakeMode() {
    driveConfig.idleMode(IdleMode.kBrake);
    angleConfig.idleMode(IdleMode.kBrake);

    driveMotor.configure(driveConfig, null, null);
    angleMotor.configure(angleConfig,  null, null);
  }

  public void setCoastMode() {
    driveConfig.idleMode(IdleMode.kCoast);
    angleConfig.idleMode(IdleMode.kCoast);

    driveMotor.configure(driveConfig,  null, null);
    angleMotor.configure(angleConfig,  null, null);

  }

}