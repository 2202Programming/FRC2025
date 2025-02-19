package frc.robot2025;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.PDPMonitorCmd;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Sensors_Subsystem;

public class RobotSpec_CompBot2025 implements IRobotSpec {
  // Subsystems and other hardware on 2025 Robot rev Competition
  final SubsystemConfig ssconfig = new SubsystemConfig("CompBot2025", "11111111") //TODO FIX serialnum
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })
      // .add(PneumaticsControl.class)
      // .add(BlinkyLights.class, "LIGHTS")
      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);
      })
    //   .add(Sensors_Subsystem.class)
    //   .add(Limelight.class)
    //  .add(SwerveDrivetrain.class, () ->{
    //       return new SwerveDrivetrain(SparkFlex.class);
    //   }) // must be after LL and Sensors
    //   .add(VisionPoseEstimator.class)
      .add(Elevator_Subsystem.class)
      .add(Command.class, "ElevatorWatcher", () -> {
        return RobotContainer.getSubsystem(Elevator_Subsystem.class).getWatcher();
      });
      // .add(Wrist.class)
      // .add(EndEffector_Subsystem.class)

      // below are optional watchers for shuffeleboard data - disable if need too.
      // .add(Command.class, "DT_Monitor", () -> {
      //   return new DTMonitorCmd();
      // });

  // set this true at least once after robot hw stabilizes
  boolean burnFlash = false;
  boolean swerve = true;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  // Chassis
  double kWheelCorrectionFactor = .957;
  double kSteeringGR = 21.42857; //Mk4i 
  double kDriveGR = 6.12; //Mk4i L3
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

  final ChassisConfig chassisConfig = new ChassisConfig(
      MperFT * (23.287 / 12.0) / 2.0, // x, based off onshape measurements
      MperFT * (23.386 / 12.0) / 2.0, // y, based off onshape measurements
      kWheelCorrectionFactor, // scale [] <= 1.0
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
      new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
  );

  public RobotSpec_CompBot2025() {
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
    return RobotContainer.getSubsystem(Sensors_Subsystem.class);
  }

  @Override
  public ChassisConfig getChassisConfig() {
    return chassisConfig;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    ModuleConfig[] modules = new ModuleConfig[4];
    /*  Set these up when we have a chassis
    modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        29, 24, 25,
        41.484)
        .setInversions(false, true, false);

    modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        30, 26, 27,
        -66.621)
        .setInversions(true, true, false);

    modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        28, 22, 23,
        24.785)
        .setInversions(false, true, false);

    modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.BackRight,
        31, 20, 21,
        -40.781)
        .setInversions(true, true, false);
*/
    return modules;
  }

  @Override
  public void setBindings() {
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");
    BindingsCompetition.ConfigureCompetition(dc);
    //BindingsOther.ConfigureOther(dc);

    // start anyting else
    new PDPMonitorCmd(); // auto scheduled, runs when disabled, moved from bindings
  }

  @Override
  public boolean burnFlash() {
    return burnFlash;
  }

  @Override
  public SendableChooser<Command> getRegisteredCommands() {
    // no robot parts to support thse now
    // return RegisteredCommands.RegisterCommands();
    return null;
  }

  @Override
  public void setDefaultCommands() {
    SwerveDrivetrain drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new RobotCentricDrive());
    }
  }

}