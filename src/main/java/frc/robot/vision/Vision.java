package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    public VisionCamera cameraFront, cameraBack;
    private EagleSwerveDrivetrain drivetrain;
    private VisionSim visionSim;

    public Vision(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionSim = new VisionSim();
        cameraFront = new VisionCamera(VisionConstants.CAMERA_FRONT_CONFIG, visionSim);
        cameraBack = new VisionCamera(VisionConstants.CAMERA_BACK_CONFIG, visionSim);
    }

    @Override
    public void periodic() {
        updateDrivetrainVision(cameraFront);
        updateDrivetrainVision(cameraBack);
    }

    private void updateDrivetrainVision(VisionCamera camera) {
        var visionEst = camera.getEstimatedGlobalPose();

        visionEst.ifPresent(
                est -> {
                    var estStdDevs = camera.getEstimationStdDevs();

                    drivetrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
    }

    public int getLastSeenTag() {
        if (cameraFront.getLastSeenTagTime() > cameraBack.getLastSeenTagTime() &&
                cameraFront.getLastSeenTag() != -1) {
            return cameraFront.getLastSeenTag();
        } else {
            return cameraBack.getLastSeenTag();
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.simulationPeriodic(drivetrain.getState().Pose);

        var debugField = visionSim.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrain.getState().Pose);
    }
}
