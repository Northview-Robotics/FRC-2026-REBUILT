package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;

public class AutoAlign extends SubsystemBase{
    private static AutoAlign align = null;
    private Drive drivetrain = Drive.getInstance();
    Map<Pose2d, Pose2d> intermediatePoseMap = new HashMap<>();
    Map<Integer, Pose2d> tagMap = new HashMap<>();

    //Commented out because all of the auto align paths haven't been made yet

    private AutoAlign(){
        intermediatePoseMap.put(AutoConstants.BlueTrenchLeft, AutoConstants.BlueTrenchLeftI);
        intermediatePoseMap.put(AutoConstants.RedTrenchLeft, AutoConstants.RedTrenchLeftI);
        intermediatePoseMap.put(AutoConstants.RedTrenchRight, AutoConstants.RedTrenchRightI);
        intermediatePoseMap.put(AutoConstants.BlueTrenchRight, AutoConstants.BlueTrenchRightI);
    }

    public Rotation2d getHubHeading(){
        Pose2d botPos = drivetrain.getPose();
        Pose2d hubPos = getHubPos();

        double targetRadians = Math.atan2(
            hubPos.getY() - botPos.getY(),
            hubPos.getX() - botPos.getX()
        );

        return new Rotation2d(targetRadians);
    }

    public Distance getHubDist(){
        Pose2d botPos = drivetrain.getPose();
        Pose2d hubPos = getHubPos();

        Distance hubDist = Meters.of(
            Math.hypot((hubPos.getX() - botPos.getX()), (hubPos.getY() - botPos.getY()))
        );
        return hubDist;
    }

    private Pose2d getHubPos(){
        Pose2d hubPos = (DriverStation.getAlliance().get() == Alliance.Blue) ? AutoConstants.HubB : AutoConstants.HubR;
        return hubPos;
    }

    //Path finding
    public void followGeneratedPath(Pose2d intermediate, Pose2d target) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivetrain.getPose(), intermediate, target);
        PathConstraints constraints = new PathConstraints(4.0, 3.0, 2 * Math.PI, 2 * Math.PI);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, target.getRotation())
        );
        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();  //Follow path
    }

    public static AutoAlign getInstance(){
        if (align == null){
            align = new AutoAlign();
        }
        return align;
    }
}