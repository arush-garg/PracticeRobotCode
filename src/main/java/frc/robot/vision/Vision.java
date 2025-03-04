package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    VisionCamera cameraRight, cameraLeft;

    Vision() {
        cameraRight = new VisionCamera(VisionConstants.CAMERA_RIGHT_CONFIG);
        cameraLeft = new VisionCamera(VisionConstants.CAMERA_LEFT_CONFIG);
    }

    @Override
    public void periodic() {
        // update drivebase odom
    }
}
