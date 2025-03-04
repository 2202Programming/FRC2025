package frc.robot2025;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot2025.Constants.CAN;
import frc.robot2025.commands.Climb;
import frc.robot2025.commands.ClimberVelMove;
import frc.robot2025.subsystems.Climber;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

//copy or extend this code for your robot - remember to override:
// TBD
//
public class RobotSpec_BotOnBoard3 implements IRobotSpec {

    // Build

    // Robot Speed Limits
    RobotLimits robotLimits = new RobotLimits(
        FeetPerSecond.of(15.0),
        DegreesPerSecond.of(360.0));

    // Chassis
    double kWheelCorrectionFactor = .995;
    double kSteeringGR = 12.8;
    double kDriveGR = 8.14;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    ChassisConfig chassisConfig = new ChassisConfig(
        MperFT * (21.516 / 12.0) / 2.0, // X offset [m]
        MperFT * (24.87 / 12) / 2.0, // Y offset [m]
        kWheelCorrectionFactor, // []
        kWheelDiameter, // [m]
        kSteeringGR, // []
        kDriveGR); // []

    // SubsystemConfig gets registered in static array to match serial number at
    // Construct call
    SubsystemConfig subsystemConfig = new SubsystemConfig("bot-On-Board-3", "0326F275")
    // deferred construction via Supplier<Object> lambda
        .add(PowerDistribution.class, "PDP", () -> {
            var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
            pdp.clearStickyFaults();
            return pdp;
        })
        // .add(PneumaticsControl.class)
        //.add(BlinkyLights.class, "LIGHTS", () -> {
        //    return new BlinkyLights(CAN.CANDLE1, CAN.CANDLE2, CAN.CANDLE2, CAN.CANDLE4);
        //})
        .add(HID_Subsystem.class, "DC", () -> {
            return new HID_Subsystem(0.3, 0.9, 0.05);
        })
        //.add(Sensors_Subsystem.class)
        // .add(Limelight.class)
        //.add(EndEffector_Subsystem.class, "endEffectorSubsystem")
        .add(Climber.class)
        //.add(GroundIntake.class)
        //.add(SwerveDrivetrain.class, () -> {
        //    return new SwerveDrivetrain(SparkFlex.class);
        //}) // must be after LL and Sensors
        //.add(GroundIntake.class)
        //.add(VisionPoseEstimator.class)
        // below are optional watchers for shuffeleboard data - disable if need too.
        //.add(Command.class, "DT_Monitor", () -> {
        //    return new DTMonitorCmd();
        //})
        ;

        public RobotSpec_BotOnBoard3() {
            // finish BetaBot's drivePIDF
            chassisConfig.drivePIDF.setIZone(0.2);
            // add the specs to the ssconfig
            subsystemConfig.setRobotSpec(this);
          }
    @Override
    public RobotLimits getRobotLimits() {
        return robotLimits;
    }

    @Override
    public IHeadingProvider getHeadingProvider() {
        return null; // no sensors in default, example for your bot's Spec:
        // return RobotContainer.getSubsystem(Sensors_Subsystem.class);
    }

    @Override
    public ChassisConfig getChassisConfig() {
        return chassisConfig;
    }

    @Override
    public ModuleConfig[] getModuleConfigs() {
        return null;
    }

    @Override
    public void setBindings() {
        // FOR BOT ON BOARD you can configure bindings directly here
        // and avoid messing with BindingsOther or Comp.
        HID_Subsystem dc = RobotContainer.getSubsystem("DC");
        if(dc.Operator() instanceof CommandXboxController operator) {

            operator.povUp().whileTrue(new ClimberVelMove(0.75));
            operator.povDown().whileTrue(new ClimberVelMove(-0.75));

            operator.rightBumper().onTrue(new Climb(3.0, 0.75));


        }
        //BindingsCompetition.ConfigureCompetition(dc);
        //BindingsOther.ConfigureOther(dc);
    }

    @Override
    public boolean burnFlash() {
        return false;
    }

    @Override
    public SendableChooser<Command> getRegisteredCommands() {
        return null;
    }

    @Override
    public void setDefaultCommands() {

    }

}
