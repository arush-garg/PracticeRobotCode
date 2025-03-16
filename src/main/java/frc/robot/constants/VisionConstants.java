


package frc.robot.constants;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


public class VisionConstants {
   public static final AprilTagFieldLayout FIELD_TAG_LAYOUT = AprilTagFieldLayout
           .loadField(AprilTagFields.k2025ReefscapeWelded);


   public static final CameraConfiguration CAMERA_FRONT_CONFIG = new CameraConfiguration()
           .withCameraName("CAMERA_FRONT")
           .withRobotToCam(new Transform3d(new Translation3d(0.35999, 0, 0.13106),
                   new Rotation3d(0.0, Units.degreesToRadians(20), 0.0)))
           .withSingleTagStdDevs(VecBuilder.fill(0.05, 0.05, 0.3))
           .withMultiTagStdDevs(VecBuilder.fill(0.05, 0.05, 0.1));


   public static final CameraConfiguration CAMERA_BACK_CONFIG = new CameraConfiguration()
           .withCameraName("CAMERA_BACK")
           .withRobotToCam(new Transform3d(new Translation3d(0.35999, 0, -0.13106),
                   new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(180))))
           .withSingleTagStdDevs(VecBuilder.fill(0.05, 0.05, 0.3))
           .withMultiTagStdDevs(VecBuilder.fill(0.05, 0.05, 0.1));


   public static class CameraConfiguration {
       public String cameraName;
       public Transform3d robotToCam;
       public Matrix<N3, N1> singleTagStdDevs;
       public Matrix<N3, N1> multiTagStdDevs;


       public CameraConfiguration() {
           this.cameraName = "";
           this.robotToCam = new Transform3d();
           this.singleTagStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
           this.multiTagStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
       }


       public CameraConfiguration(String cameraName, Transform3d robotToCam, Matrix<N3, N1> singleTagStdDevs,
               Matrix<N3, N1> multiTagStdDevs) {
           this.cameraName = cameraName;
           this.robotToCam = robotToCam;
           this.singleTagStdDevs = singleTagStdDevs;
           this.multiTagStdDevs = multiTagStdDevs;
       }


       public CameraConfiguration withSingleTagStdDevs(Matrix<N3, N1> stdDevs) {
           return new CameraConfiguration(cameraName, robotToCam, stdDevs, multiTagStdDevs);
       }


       public CameraConfiguration withMultiTagStdDevs(Matrix<N3, N1> stdDevs) {
           return new CameraConfiguration(cameraName, robotToCam, singleTagStdDevs, stdDevs);
       }


       public CameraConfiguration withRobotToCam(Transform3d robotToCam) {
           return new CameraConfiguration(cameraName, robotToCam, singleTagStdDevs, multiTagStdDevs);
       }


       public CameraConfiguration withCameraName(String cameraName) {
           return new CameraConfiguration(cameraName, robotToCam, singleTagStdDevs, multiTagStdDevs);
       }
   }
}


 