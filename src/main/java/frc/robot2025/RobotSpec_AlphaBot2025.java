package frc.robot2025;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.PDPMonitorCmd;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.BlinkyLights;
import frc.lib2202.subsystem.Odometry;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.VisionPoseEstimator;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.AutoPPConfigure;
import frc.lib2202.subsystem.swerve.DTMonitorCmd;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.commands.*;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.GroundIntake;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.robot2025.subsystems.Limelight;
import frc.robot2025.subsystems.Sensors_Subsystem;
import frc.robot2025.subsystems.Wrist;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Sensors_Subsystem;

public class RobotSpec_AlphaBot2025 implements IRobotSpec {
  // Subsystems and other hardware on 2025 Robot rev Alpha
  // $env:serialnum = "03282B65"
  final SubsystemConfig ssconfig = new SubsystemConfig("AlphaBot2025", "03282B65")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      // .add(PneumaticsControl.class)
      .add(BlinkyLights.class, "LIGHTS", () -> {
        return new BlinkyLights(CAN.CANDLE1, CAN.CANDLE2, CAN.CANDLE3, CAN.CANDLE4);
      })
      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);
      })
      // .add(GroundIntake.class)
      .add(Elevator_Subsystem.class)
      .add(Command.class, "ElevatorWatcher", () -> {
       return RobotContainer.getSubsystem(Elevator_Subsystem.class).getWatcher();
      })

      // Sensors, limelight and drivetrain all use interfaces, so make sure their alias names
      // match what is given here.
      .add(Sensors_Subsystem.class, "sensors")
      .add(Limelight.class, "limelight")
      .add(SwerveDrivetrain.class, "drivetrain", () ->{
          return new SwerveDrivetrain(SparkFlex.class);
      })
      .add(OdometryInterface.class, "odometry", () ->{
        var obj = new Odometry();
        obj.new OdometryWatcher();
        return obj;
      })
      // VisonPoseEstimator needs LL and Odometry
      .add(VisionPoseEstimator.class)
      // below are optional watchers for shuffeleboard data - disable if need too.
      .add(Command.class, "DT_Monitor", () -> {
        return new DTMonitorCmd();
      })
      .add(EndEffector_Subsystem.class, "endEffectorSubsystem")
      .add(Command.class, "endEffectorWatcher", () -> {
        return ((EndEffector_Subsystem)RobotContainer.getSubsystem("endEffectorSubsystem")).getWatcher();
      })
      ;
      // .add(Wrist.class);

  boolean swerve = true;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  // Chassis
  double kWheelCorrectionFactor = 1.02;
  double kSteeringGR = 21.428;
  double kDriveGR = 6.12;
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

  final ChassisConfig chassisConfig = new ChassisConfig(
      0.57785 / 2.0, // x, based on direct measurements
      0.57785 / 2.0, // y, based on direct measurements
      kWheelCorrectionFactor, // scale [] <= 1.0
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
      new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
  );

  public RobotSpec_AlphaBot2025() {
    // finish BetaBot's drivePIDF
    chassisConfig.drivePIDF.setIZone(0.2);
    // add the specs to the ssconfig
    ssconfig.setRobotSpec(this);
  }

  // Required method that use the specs above

  @Override
  public RobotLimits getRobotLimits() {
    return robotLimits;
  }

  @Override
  public IHeadingProvider getHeadingProvider() {
    return RobotContainer.getSubsystem("sensors");
  }

  @Override
  public ChassisConfig getChassisConfig() {
    return chassisConfig;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    ModuleConfig[] modules = new ModuleConfig[4];
    modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        CAN.FL_CANCoder, CAN.FL_Drive, CAN.FL_Angle,
        43.41759375)
        .setInversions(false, true, false);

    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        CAN.FR_CANCoder, CAN.FR_Drive, CAN.FR_Angle,
        -66.2694375)
        .setInversions(true, true, false);

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        CAN.BL_CANCoder, CAN.BL_Drive, CAN.BL_Angle,
        49.482265625)
        .setInversions(false, true, false);

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        CAN.BR_CANCoder, CAN.BR_Drive, CAN.BR_Angle,
        -66.005609375)
        .setInversions(true, true, false);

    return modules;
  }

  // @Override
  // public void setBindings() {
  //   HID_Subsystem dc = RobotContainer.getSubsystem("DC");

  //   // TODO - figure better way to handle bindings
    
  //   // Select either comp or other for testing
  //   BindingsCompetition.ConfigureCompetition(dc);
  //   //BindingsOther.ConfigureOther(dc);

  //   // Initialize PathPlanner
    // OdometryInterface odo = RobotContainer.getSubsystemOrNull("odometry");
    // DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
    // if (odo != null && sdt != null) {
      // AutoPPConfigure.configureAutoBuilder(sdt, odo);
  //   }

  //   // start anyting else
  //   new PDPMonitorCmd(); // auto scheduled, runs when disabled, moved from bindings
  // }

  @Override
  public boolean burnFlash() {
    return true;
  }

 @Override
  public void setBindings() {
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");
    if (dc.Driver() instanceof CommandPS4Controller) {
      // CommandPS4Controller opp = (CommandPS4Controller)dc.Driver();

      // opp.square().onTrue(new WristToPos(1.0));
      // opp.triangle().onTrue(new WristToPos(0.0));
      // opp.cross().onTrue(new WristToPos(0.5));
    } else {
      CommandXboxController driver = (CommandXboxController)dc.Driver();
      CommandXboxController opp = (CommandXboxController)dc.Operator();
      OdometryInterface odo = RobotContainer.getSubsystemOrNull("odometry");
      DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
      if (odo != null && sdt != null) {
      AutoPPConfigure.configureAutoBuilder(sdt, odo);
      }
                  driver.rightTrigger().whileTrue(new RobotCentricDrive(sdt, dc));
            driver.y().onTrue(new AllianceAwareGyroReset(true));
      final Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
      opp.x().whileTrue(new testElevatorVelComd(30.0));
      opp.a().onTrue(new ElevatorCalibrate(-30.0));

      opp.y().onTrue(new ClimberSetPos(0.0));
      // opp.b().onTrue(new InstantCommand(() -> {
      //   elevator_Subsystem.setHeight(50.0);
      // }));
      // opp.a().onTrue(new InstantCommand(() -> {
      //   elevator_Subsystem.setHeight(90.0);
      // }));
      opp.povDown().onTrue(new InstantCommand(() -> {
        elevator_Subsystem.setHeight(46.0);
      }));
      opp.povLeft().onTrue(new InstantCommand(() -> {
        elevator_Subsystem.setHeight(86.0);
      }));
      opp.povRight().onTrue(new InstantCommand(() -> {
        elevator_Subsystem.setHeight(0.0);
      }));
      // opp.rightTrigger().onTrue(new InstantCommand(() -> {
      //   elevator_Subsystem.setHeight(75.0);
      // }));
      opp.rightBumper().whileTrue(new EndEffectorPercent(-0.7, "rightBumper")); //reverse
      opp.rightTrigger().whileTrue(new EndEffectorPercent(.3, "rightTrigger"));
      //for end effector
      //opp.rightBumper().whileTrue(new EndEffectorPercent(-.3, "rightBumper")); //reverse
      //opp.rightTrigger().whileTrue(new EndEffectorPercent(.5, "rightTrigger")); //p
      
      // opp.x().whileTrue(new backupEE_Move(1000.0)); 
    }

    

    // FOR BOT ON BOARD you can configure bindings directly here
    // and avoid messing with BindingsOther or Comp.

    //BindingsOther.ConfigureOther(dc);
  }



  @Override
  public SendableChooser<Command> getRegisteredCommands() {
    // no robot parts to support thse now
    return RegisteredCommands.RegisterCommands();
  }

  @Override
  public void setDefaultCommands() {
    DriveTrainInterface drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive());
    }
  }

}