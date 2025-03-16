package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    VisionCamera cameraFront, cameraBack;
    EagleSwerveDrivetrain drivetrain;
    ElasticSender m_elastic;

    public Vision(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        cameraFront = new VisionCamera(VisionConstants.CAMERA_FRONT_CONFIG);
        cameraBack = new VisionCamera(VisionConstants.CAMERA_BACK_CONFIG);
        m_elastic = new ElasticSender("vision", true);
        
    }

    @Override
    public void periodic() {
        updateDrivetrainVision(cameraFront);
        updateDrivetrainVision(cameraBack);
        m_elastic.put("front camera", cameraFront.getEstimatedGlobalPose().toString(), false);
        m_elastic.put("back camera", cameraBack.getEstimatedGlobalPose().toString(), false);


        m_elastic.put("front camera std", cameraFront.getEstimationStdDevs() != null ? cameraFront.getEstimationStdDevs().get(0, 0) : "null", false);
        m_elastic.put("back camera std", cameraBack.getEstimationStdDevs() != null ? cameraBack.getEstimationStdDevs().toString() : "null", false);

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
