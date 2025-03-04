package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    VisionCamera cameraRight, cameraLeft;
    EagleSwerveDrivetrain drivetrain;

    public Vision(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        cameraRight = new VisionCamera(VisionConstants.CAMERA_RIGHT_CONFIG);
        cameraLeft = new VisionCamera(VisionConstants.CAMERA_LEFT_CONFIG);
    }

    @Override
    public void periodic() {
        updateDrivetrainVision(cameraRight);
        updateDrivetrainVision(cameraLeft);
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

    public int getCurrentlySeenTag() {
        return cameraRight.getCurrentlySeenTag() != 0 ? cameraRight.getCurrentlySeenTag()
                : cameraLeft.getCurrentlySeenTag();
    }
}
