package frc.robot2025;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.ElevatorMove;
import frc.robot2025.commands.testElevatorVelComd;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

/**
 * NOAH COMMENTS ON TESTING ELEVATOR AND END EFFECTOR FOR 2/22
 * For tuning velocity PID (make sure to re-tune), run: TestElevatorVelCmd -- it uses SmartDashboard (Current vel)
 * For moving to various positions on elevator: Use instant command, copy paste below and change position
 * ElevatorCalibrate to calibrate Elevator
 * op.b().onTrue(new InstantCommand(() -> {
        elevator_Subsystem.setHeight(50.0);
      }));
 * Use WristToPos to test wrist (Linear servo or actuator or wtv it's being called, the thing that moves EE between 2 pos)
 * Use EndEffectorRPM to test RPM of EndEffector
 * Test various sequences: PickupSequence & DropSequence
 * PickupSequence: Look at the documentation for params. For either picking up a piece from ground
 *  (if possible, if not then just remove boolean), or getting it from station
 * DropSequence: To drop the piece onto whatever level you're aiming for
/*

 * Bindings here for testing, 
 */
public class BindingsOther {
    // enum for bindings add when needed
    public enum Bindings {
        DriveTest, ElevatorTesting
    }

    static Bindings bindings = Bindings.DriveTest;

    public static void ConfigureOther(HID_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }

    // wrap the pathloading with try/catch
    static PathPlannerPath loadFromFile(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            return PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Big oops loading pn="+ pathName + ":" + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    // Warning, these will fail if XBOX controllers are not connected
    static void DriverBinding(HID_Subsystem dc) {        
        var generic_driver = dc.Driver();
        DriveTrainInterface drivetrain = RobotContainer.getSubsystem("drivetrain");
        CommandXboxController driver = (CommandXboxController)generic_driver;

        // keep LB and y same even in testing
        driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.y().onTrue(new AllianceAwareGyroReset(false));
        
        switch (bindings) {
            // add any test binding cases here
            default:
                break;
        }
    }
    
    // Warning, these will fail if XBOX controllers are not connected
    static void OperatorBindings(HID_Subsystem dc) {
        CommandXboxController operator = (CommandXboxController)dc.Operator();

        switch (bindings) {
            case ElevatorTesting:
                ((CommandXboxController) operator).x().whileTrue(new testElevatorVelComd(2430.0));
                ((CommandXboxController) operator).a().whileTrue(new testElevatorVelComd(100.0));
                ((CommandXboxController) operator).y().whileTrue(new testElevatorVelComd(-2430.0));
                ((CommandXboxController) operator).b().onTrue(new ElevatorMove(Levels.Ground));
                ((CommandXboxController) operator).rightTrigger().onTrue(new ElevatorMove(Levels.LCoral));
            default:
                break;
        }
    }

}
