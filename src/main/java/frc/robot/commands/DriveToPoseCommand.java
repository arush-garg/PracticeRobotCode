package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class DriveToPoseCommand extends Command {
    private final EagleSwerveDrivetrain drivetrain;

    private Pose2d targetPose;
    private Command pathCommand;

    public DriveToPoseCommand(EagleSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        this.targetPose = targetPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startPath();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.cancel();
            System.out.println("DriveToPoseCommand finished");
        }
    }

    private void startPath() {
        if (targetPose != null) {
            Pose2d currentPose = drivetrain.getState().Pose;

            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double angleDifference = Math
                    .abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees());

            if (distance < 0.05 && angleDifference < 2) { // 5 cm and 2 degrees tolerance
                System.out.println("Already at target pose");
                pathCommand = null;
                return;
            }

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);

            if (waypoints.isEmpty()) {
                System.out.println("No waypoints generated");
                pathCommand = null;
                return;
            }

            System.out.println("Waypoints: " + waypoints.size());

            System.out.println("Waypoints first: " + waypoints.get(0).toString());

            PathPlannerPath generatedPath = new PathPlannerPath(waypoints, AutoAlignConstants.PATH_PLANNER_CONSTRAINTS,
                    null,
                    new GoalEndState(0, targetPose.getRotation()));
            generatedPath.preventFlipping = true;
            pathCommand = AutoBuilder.followPath(generatedPath);

            if (pathCommand == null) {
                System.out.println("PathPlanner failed to generate a command");
                return;
            }

            try {
                pathCommand.initialize();
            } catch (Exception e) {
                e.printStackTrace();
                return;
            }
        }
    }
}
