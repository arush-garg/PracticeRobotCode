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
import frc.robot.vision.Vision;

public class DriveToPoseCommand extends Command {
    private final EagleSwerveDrivetrain drivetrain;
    private final Vision vision;

    public enum TargetPosition {
        LEFT, RIGHT, ALGAE
    }

    private final TargetPosition targetPosition;

    private Pose2d targetPose;
    private Command pathCommand;
    private int aprilTagId;

    public DriveToPoseCommand(EagleSwerveDrivetrain drivetrain, Vision vision, TargetPosition targetPosition) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetPosition = targetPosition;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        updateTargetPose();
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

    private void updateTargetPose() {
        if (targetPose != null && drivetrain.getState().Pose.equals(targetPose)) {
            System.out.println("Already at target pose");
            pathCommand = null;
            return;
        }

        int currentAprilTagId = vision.getCurrentlySeenTag();

        if (currentAprilTagId != -1) {
            aprilTagId = currentAprilTagId;
        }

        targetPose = getTargetPose(aprilTagId);

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
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

    private Pose2d getTargetPose(int aprilTagId) {
        return switch (targetPosition) {
            case LEFT -> switch (aprilTagId) {
                case 18, 7 -> AutoAlignConstants.REEF_A;
                case 19, 6 -> AutoAlignConstants.REEF_K;
                case 20, 11 -> AutoAlignConstants.REEF_I;
                case 21, 10 -> AutoAlignConstants.REEF_G;
                case 22, 9 -> AutoAlignConstants.REEF_E;
                case 17, 8 -> AutoAlignConstants.REEF_C;
                default -> {
                    System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            case RIGHT -> switch (aprilTagId) {
                case 18, 7 -> AutoAlignConstants.REEF_B;
                case 19, 6 -> AutoAlignConstants.REEF_L;
                case 20, 11 -> AutoAlignConstants.REEF_J;
                case 21, 10 -> AutoAlignConstants.REEF_H;
                case 22, 9 -> AutoAlignConstants.REEF_F;
                case 17, 8 -> AutoAlignConstants.REEF_D;
                default -> {
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            case ALGAE -> switch (aprilTagId) {
                case 18, 7 -> AutoAlignConstants.ALGAE_AB;
                case 19, 6 -> AutoAlignConstants.ALGAE_KL;
                case 20, 11 -> AutoAlignConstants.ALGAE_IJ;
                case 21, 10 -> AutoAlignConstants.ALGAE_GH;
                case 22, 9 -> AutoAlignConstants.ALGAE_EF;
                case 17, 8 -> AutoAlignConstants.ALGAE_CD;
                default -> {
                    System.out.println("Unknown AprilTag ID for algae: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
        };
    }
}
