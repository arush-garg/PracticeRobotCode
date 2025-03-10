package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    VisionCamera cameraFront, cameraBack;
    EagleSwerveDrivetrain drivetrain;

    public Vision(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        cameraFront = new VisionCamera(VisionConstants.CAMERA_FRONT_CONFIG);
        cameraBack = new VisionCamera(VisionConstants.CAMERA_BACK_CONFIG);
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

    public int getCurrentlySeenTag() {
        return cameraFront.getCurrentlySeenTag() != -1 ? cameraFront.getCurrentlySeenTag()
                : cameraBack.getCurrentlySeenTag();
    }
}
