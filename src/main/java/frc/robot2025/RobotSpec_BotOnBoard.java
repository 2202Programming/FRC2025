package frc.robot2025;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

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
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.commands.ElevatorCalibrate;
import frc.robot2025.commands.ElevatorMove;
import frc.robot2025.commands.EndEffectorRPM;
import frc.robot2025.commands.WristToPos;
import frc.robot2025.commands.backupEE_Move;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.robot2025.subsystems.Sensors_Subsystem;
import frc.robot2025.subsystems.Wrist;
import frc.robot2025.subsystems.backupEE;

public class RobotSpec_BotOnBoard implements IRobotSpec {
  // Subsystems and other hardware on 2025 Robot rev Alpha
  // $env:serialnum = "032381BF"
  final SubsystemConfig ssconfig = new SubsystemConfig("BotOnBoard", "0312db1a")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      })

      .add(HID_Subsystem.class, "DC", () -> {
        return new HID_Subsystem(0.3, 0.9, 0.05);

      })
      
      // .add(Wrist.class);
      .add(EndEffector_Subsystem.class)
      .add(Command.class, "EndEffectorWatcher", () -> {
       return RobotContainer.getSubsystem(EndEffector_Subsystem.class).getWatcher();
      });
      // below are optional watchers for shuffeleboard data - disable if need too.

  // set this true at least once after robot hw stabilizes
  boolean burnFlash = false;
  boolean swerve = true;

  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  // Chassis
  double kWheelCorrectionFactor = .957;
  double kSteeringGR = 21.428;
  double kDriveGR = 6.12;
  double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

  final ChassisConfig chassisConfig = new ChassisConfig(
      MperFT * (25.0 / 12.0) / 2.0, // x
      MperFT * (20.75 / 12.0) / 2.0, // y
      kWheelCorrectionFactor, // scale [] <= 1.0
      kWheelDiameter,
      kSteeringGR,
      kDriveGR,
      new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
      new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
  );

  public RobotSpec_BotOnBoard() {
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
    // bot on board, no modules
    ModuleConfig[] modules = null;
    return modules;
  }

  @Override
  public void setBindings() {
    HID_Subsystem dc = RobotContainer.getSubsystem("DC");
    if (dc.Driver() instanceof CommandPS4Controller) {
      CommandPS4Controller opp = (CommandPS4Controller)dc.Driver();

      // opp.square().onTrue(new WristToPos(1.0));
      // opp.triangle().onTrue(new WristToPos(0.0));
      // opp.cross().onTrue(new WristToPos(0.5));
    } else {
      CommandXboxController opp = (CommandXboxController)dc.Driver();
      final Elevator_Subsystem elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
      opp.b().whileTrue(new EndEffectorRPM(500.0));
      opp.a().whileTrue(new EndEffectorRPM(1000.0));
      opp.y().whileTrue(new EndEffectorRPM(2000.0));
    }

    

    // FOR BOT ON BOARD you can configure bindings directly here
    // and avoid messing with BindingsOther or Comp.

    //BindingsOther.ConfigureOther(dc);
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
  }

}