package frc.robot2025;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
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
import frc.robot2025.commands.DriveToReefTag;
// 2024 robot has a pigeon, so use its sensors, add LL4
import frc.robot2025.subsystems.Limelight;
import frc.robot2025.subsystems.Sensors_Subsystem;


public class RobotSpec_test2024 implements IRobotSpec {

    /*
     * This is a stripped down spec to allow driving 2024 comp bot on
     * only the 2025 or library softare.  
     * 
     * THis is so we can use it for drive practice or testing 2025 pathing
     * while the 2025 bot is being worked ont
     */

    // CAN values for this bot, copied from constants.java
    public static final class CAN {
        public static final int ROBORIO = 0;
        public static final int PDP = 1; // for rev
        public static final int PCM1 = 2; // for rev
    
        // lights
        public static final int CANDLE1 = 3;
        public static final int CANDLE2 = 4;

        //PIGEON is on 60 here too, picked up from 2025 constants.
    }

    boolean teleOpRunOnce = true;

    final SubsystemConfig config = new SubsystemConfig(
        // Subsystems and other hardware for testing with 2024 comp-bot
        // $env:serialnum = "032D2062"
        "CompetitionBot2024 as testing platform.", "032D2062")
        // deferred construction via Supplier<Object> lambda
        .add(PowerDistribution.class, "PDP", () -> {
                var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
                pdp.clearStickyFaults();
                return pdp;
            })
            .add(BlinkyLights.class, "LIGHTS", () -> {
                return new BlinkyLights(CAN.CANDLE1, CAN.CANDLE2);
            })
            .add(HID_Subsystem.class, "DC", () -> {
                return new HID_Subsystem(0.3, 0.9, 0.05);
            })

            // using same setup as 2025 comp, not same as orginal 2024
            .add(Sensors_Subsystem.class, "sensors")     //2025
            .add(Limelight.class, "limelight")           //2025  - added LL4 for testing
            .add(SwerveDrivetrain.class, "drivetrain", () -> {
                return new SwerveDrivetrain();  //2024 sdt, no sparkflex
            })
            .add(OdometryInterface.class, "odometry", () -> {
                var obj = new Odometry();
                obj.new OdometryWatcher();
                return obj;
            })
            // VisonPoseEstimator needs LL and Odometry
            .add(VisionPoseEstimator.class)
            .add(Command.class, "DT_Monitor", () -> {
                return new DTMonitorCmd();
            });

    // set this true at least once after robot hw stabilizes
    boolean burnFlash = false;
    boolean swerve = true;

    // Robot Speed Limits
    double maxSpeedFPS = 15.0; // [ft/s] 
    double maxRotationRateDPS = 360.0; // [deg/s]
    RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));
    
    // Chassis
    double kWheelCorrectionFactor = .987;
    double kSteeringGR = 21.428;
    double kDriveGR = 6.12;
    double kWheelDiameter = MperFT * 4.0 / 12.0; // [m]

    final ChassisConfig comp2024BotBetaChassisConfig = new ChassisConfig(
            MperFT * (24.875 / 12.0) / 2.0, // x
            MperFT * (20.5 / 12.0) / 2.0, // y
            kWheelCorrectionFactor, // scale [] <= 1.0
            kWheelDiameter,
            kSteeringGR,
            kDriveGR,
            new PIDFController(0.085, 0.00055, 0.0, 0.21292), // drive
            new PIDFController(0.01, 0.0, 0.0, 0.0) // angle
    );

    public RobotSpec_test2024() {
        // finish BetaBot's drivePIDF
        comp2024BotBetaChassisConfig.drivePIDF.setIZone(0.2);

        // finally add this spec to the config
        config.setRobotSpec(this);
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
        return comp2024BotBetaChassisConfig;
    }

    @Override
    /// swapped corner Ids for matching 2025 Limelight placement.
    /// LL is on Front Left corner
    public ModuleConfig[] getModuleConfigs() {
        ModuleConfig[] modules = new ModuleConfig[4];
        
        modules[CornerID.FrontLeft.getIdx()] = new ModuleConfig(CornerID.BackRight,
        28, 22, 23,
        28.125 + 90.0)
        .setInversions(false, true, false);

        modules[CornerID.FrontRight.getIdx()] = new ModuleConfig(CornerID.BackLeft,
        29, 24, 25,
        -125.595 + 90.0)
        .setInversions(false, true, false);
        
        modules[CornerID.BackLeft.getIdx()] = new ModuleConfig(CornerID.FrontRight,
        31, 20, 21,
        -115.752 + 90.0)
        .setInversions(true, true, false);

        modules[CornerID.BackRight.getIdx()] = new ModuleConfig(CornerID.FrontLeft,
        30, 26, 27,
        -114.785 + 90.0)
        .setInversions(true, true, false);
                
        return modules;
    }

    @Override
    public void setBindings() {
        // Initialize PathPlanner
        OdometryInterface odo = RobotContainer.getSubsystemOrNull("odometry");
        DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
        if (odo != null && sdt != null) {
            AutoPPConfigure.configureAutoBuilder(sdt, odo);
        }

        HID_Subsystem dc = RobotContainer.getSubsystem("DC");


        var generic_driver = dc.Driver();

        if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController)generic_driver;
            driver.rightTrigger().whileTrue(new RobotCentricDrive(sdt, dc));
            driver.y().onTrue(new AllianceAwareGyroReset(true));

           // driver.rightTrigger().whileTrue(new TargetCentricDrive(Tag_Pose.ID4, Tag_Pose.ID7));
        }
        else {
            DriverStation.reportWarning("Bindings: No driver bindings set, check controllers.", false);
        }

        // Keep binding in this file please - this bot is only for driver or path testing.
        @SuppressWarnings("unused")  //wip
        Command  test = new DriveToReefTag("l");

    }


    @Override
    public SendableChooser<Command> getRegisteredCommands() {
        // configure pathplanner and the Registered commands
        // ConfigureAutobuilder uses default pids and looks up SwereveDrivetrain from RobotContainer
        ///AutoPPConfig.ConfigureAutoBuilder();
        /// 
        return null; //RegisteredCommands.RegisterCommands();
    }

    @Override
    public void setDefaultCommands() {
        DriveTrainInterface drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
        if (drivetrain != null) {
            drivetrain.setDefaultCommand(new FieldCentricDrive());        
        }
    }
    @Override
    public void teleopInit(){
         // Temp command for compbot2024 to calibrate shooter's servo
        if (teleOpRunOnce) {
            // ensure shooter is calibrated on power up - note for a competition this
            // should not be needed and the bot should be calibrated in the pit
            //var cmd = new CalibrateWithLS();
            //cmd.schedule();
            teleOpRunOnce = false;
        }
    }
}
