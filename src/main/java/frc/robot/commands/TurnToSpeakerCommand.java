/*TO-DO:
 1. make sure it rotates the right way
 2. proportional multiplier for rot speed in driveSubsystem.drive calls
 3. tune the accepted error for yawDiff (in the if else if conditions)
 4. get robot offset constants accounted for in the relativeX and relativeZ
 5. see how much in execute can be moved to initialize to optimize speed and necessary processing power
 6. figure out what's being weird in RobotContainer if at all possible
*/
package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.VisionConstants;

public class TurnToSpeakerCommand extends Command{
    private final Vision vision;
    private final DriveSubsystem driveSubsystem;
    //these will be the same target. However they are different data types and we need some functions
    //that are exclusive to each so they have the same target but technically are different
    //blame photonlib not us
    Transform3d targetToTrack;
    PhotonTrackedTarget idealTarget;
    double yawDiff;
   

    public TurnToSpeakerCommand(Vision inputVision, DriveSubsystem inputDriveSubsystem) {
        vision = inputVision;
        driveSubsystem = inputDriveSubsystem;
        addRequirements(vision);
    } 

    @Override
    public void initialize() {   
    }

    //still to-do: implement tag changing based on alliance - not a priority for now this early!!!
    @Override
    public void execute() {
        PhotonCamera camera = vision.getCamera();
        var result = camera.getLatestResult();
        //if there are targets, iterates through the list to find target 7 and sets targetToTrack equal to that target
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() == 7) {
                    targetToTrack = target.getBestCameraToTarget();
                }
            }
            //if it didn't find target 7, tries the same thing but with target 8. if neither works, nothing happens
            if (targetToTrack == null) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == 8) {
                        targetToTrack = target.getBestCameraToTarget();
                        idealTarget = target;
                    }
                }
            }
            if (targetToTrack == null) {
                System.out.println("Cannot see applicable targets to track based on. Please reposition robot. Command ending.");
                yawDiff = 0;
                return;
            }
        }
        //gets the x and z pos relative to target 7 (or 8) and calculates desired yaw
        //desired yaw is the angle between the robot center and imaginary line through center of target
        double relativeX = targetToTrack.getX();
        double relativeZ = targetToTrack.getZ();
        double desiredYaw = Math.atan(relativeX/relativeZ);
        double actualYaw = idealTarget.getYaw();
        yawDiff = desiredYaw-actualYaw;
        //the way the yaw is calculated is not yet known. this may turn the wrong way. sorry.
        //also it may move way too fast or way too slow. sorry.
        if (yawDiff > 1) {
            driveSubsystem.drive(0,0,0.1,true,true);
        } else if (yawDiff < -1) {
            driveSubsystem.drive(0,0,0.1,true,true);
        } else if (yawDiff < 1 && yawDiff > -1) {
            yawDiff = 0;
        }


    }

    @Override
    public boolean isFinished() {
        if (yawDiff == 0) {
            return true;
        }
        else {
        return false;
    }
    }
}
