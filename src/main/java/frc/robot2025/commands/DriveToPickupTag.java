package frc.robot2025.commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.OdometryInterface;
import frc.robot2025.Constants.TheField;
import frc.robot2025.subsystems.Limelight;
import frc.robot2025.utils.UXTrim;

public class DriveToPickupTag extends Command{
    static double LeftOffset = 0.0;    //[m]
    static double RightOffset = 0.0;   //[m]
    static double BackupOffset = 0.55; //[m]

    static PathConstraints constraints = new PathConstraints(2.5, 1.75, Math.PI, Math.PI / 2.0);

    final Limelight LL;
    final String LLName;
    final OdometryInterface odo;
    final String odoName = "vision_odo";   //todo make an arg
    final int tagIdx;
    final UXTrim backoffTrim;
    final UXTrim xyTrim;
    final double xyOffset;
    final String side;
    
    // command vars set at init time
    boolean done;
    Pose2d target;
    Command moveComand;
    public DriveToPickupTag(String side){
        odo = RobotContainer.getSubsystemOrNull(odoName);
        LL = RobotContainer.getObjectOrNull("limelight");
        LLName = (LL != null) ? LL.getName() : "no-ll-found";  //name if we need to use LLHelpers directly
        this.side = side.toLowerCase();

        // pick a direction to go, left , right in TheField
        tagIdx = this.side.startsWith("l") ? 1 : 0; 
        xyOffset = this.side.startsWith("l") ? LeftOffset : RightOffset;

        backoffTrim = new UXTrim("Pickup_backoff_" + side, 0.0);
        xyTrim = new UXTrim("Pickup_xy_" + side, 0.0);
    }

    @Override
    public void initialize() {
        done = true;
        //protect from missng required ss
        if (LL == null) return;
        if (odo == null) return;
        moveComand = null;       

        Alliance alliance;
        // get our alliance
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();            
        }
        else return; // no alliance, bail

         // set LL targets to our reef only
        int tagId = (alliance == Alliance.Blue) ? TheField.PickupIdsBlue[tagIdx] : TheField.PickupIdsRed[tagIdx];
        var p3d = TheField.fieldLayout.getTagPose(tagId).get();

        var rot2d = p3d.getRotation().toRotation2d();//.getAngle();
        // left/right shift - rotate tag vector 90 outbound, use UXTrim
        var lrRot = rot2d.plus( side.startsWith("l") ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
        double lr_dx = lrRot.getCos() * xyTrim.getValue(xyOffset);
        double lr_dy = lrRot.getSin() * xyTrim.getValue(xyOffset);

        // Backup robot along tag face, use trims to adjust
        double dx = rot2d.getCos() * backoffTrim.getValue(BackupOffset);
        double dy = rot2d.getSin() * backoffTrim.getValue(BackupOffset);

        //rotate tag vector to calc L/R shift
        target = new Pose2d(p3d.getX() + dx + lr_dx, p3d.getY() + dy + lr_dy, rot2d);

        moveComand = new MoveToPose(odoName, constraints, target);
        moveComand.schedule();           
        //made it this far, start looking for reef tages in execute()
        done = false;        
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
           if (moveComand != null && moveComand.isScheduled()) {
             moveComand.cancel();
           }
        }
    }

    @Override
    public boolean isFinished() {
        if (moveComand != null) {
            done = moveComand.isFinished();
        }
        return done;
    }

}
