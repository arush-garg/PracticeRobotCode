package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.EndEffectorWristSide;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    public VisionCamera cameraFront, cameraBack;
    private EagleSwerveDrivetrain drivetrain;
    private ElasticSender m_elastic;

    public Vision(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        cameraFront = new VisionCamera(VisionConstants.CAMERA_FRONT_CONFIG);
        cameraBack = new VisionCamera(VisionConstants.CAMERA_BACK_CONFIG);
        //m_elastic = new ElasticSender("vision", true);
    }

    @Override
    public void periodic() {
        updateDrivetrainVision(cameraFront);
        updateDrivetrainVision(cameraBack);
        // m_elastic.put("front camera",
        // cameraFront.getEstimatedGlobalPose().toString(), false);
        // m_elastic.put("back camera", cameraBack.getEstimatedGlobalPose().toString(),
        // false);

        /*m_elastic.put("front camera std",
                cameraFront.getEstimationStdDevs() != null ? cameraFront.getEstimationStdDevs().get(0, 0) : "null",
                false);
        m_elastic.put("back camera std",
                cameraBack.getEstimationStdDevs() != null ? cameraBack.getEstimationStdDevs().toString() : "null",
                false);*/

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
        //return cameraFront.getLastSeenTag();
        System.out.println("front tag: " + cameraFront.getLastSeenTag() + "  back tag: " + cameraBack.getLastSeenTag());
        System.out.println("front time: " + cameraFront.getLastSeenTagTime() + "  back time: " + cameraBack.getLastSeenTagTime());
        System.out.println("front is bigger time? " + (cameraFront.getLastSeenTagTime() > cameraBack.getLastSeenTagTime()));
        if ( cameraFront.getLastSeenTagTime() > cameraBack.getLastSeenTagTime()) {
            /*if ((cameraFront.getLastSeenTag() >= 6 && cameraFront.getLastSeenTag() <= 11) || (cameraFront.getLastSeenTag() >= 17 && cameraFront.getLastSeenTag() <= 22))  {
                System.out.println("correct tag");
            }*/
            
            System.out.println("sending front");
            return cameraFront.getLastSeenTag();
        }
        else {
            System.out.println("sending back");
            return cameraBack.getLastSeenTag();
        }
        // if (cameraFront.getLastSeenTagTime() > cameraBack.getLastSeenTagTime() && cameraFront.getLastSeenTag() != -1) {
        //     return cameraFront.getLastSeenTag();
        // } else {
        //     return cameraBack.getLastSeenTag();
        // }
    }

    public EndEffectorWristSide getScoringSide() {
        if ( cameraFront.getLastSeenTagTime() > cameraBack.getLastSeenTagTime()) {
            //System.out.println("sending front");
            return EndEffectorWristSide.FRONT;
        }
        else {
            //System.out.println("sending back");
            return EndEffectorWristSide.BACK;
        }
    }

    @Override
    public void simulationPeriodic() {
        cameraFront.simulationPeriodic(drivetrain.getState().Pose);
        cameraBack.simulationPeriodic(drivetrain.getState().Pose);

        var debugField = cameraFront.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrain.getState().Pose);

        debugField = cameraBack.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrain.getState().Pose);
    }
}
