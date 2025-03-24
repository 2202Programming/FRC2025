package frc.robot2025.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.robot2025.commands.Climber.Climb;
import frc.robot2025.commands.Climber.ClimberPosition;
import frc.robot2025.commands.Climber.ClimberVelMove;
import frc.robot2025.subsystems.Climber;

/** Add your docs here. */
public class BGClimberTest {
    static Climber climber;

    static final double position = 3.0; // somewhat tested value

    public static void myBindings(HID_Subsystem dc) {
        climber = RobotContainer.getObjectOrNull("climber");

        CommandXboxController operator = (dc.Operator() instanceof CommandXboxController)
                                       ? (CommandXboxController) dc.Operator()
                                       : null;
        
        if(climber != null && operator != null) {
            xboxOperator(operator);
        }
    }


    // GEAR RATIO: 180
    static void xboxOperator(CommandXboxController operator) {
        operator.povUp().whileTrue(new ClimberVelMove(90)); // [mtr rots/min] | 0.5 rot/s
        operator.povDown().whileTrue(new ClimberVelMove(-90)); // [mtr rots/min] | 0.5 rot/s

        operator.a().onTrue(new ClimberPosition(position));

        //sequence
        operator.b().onTrue(new Climb());
    }
}
