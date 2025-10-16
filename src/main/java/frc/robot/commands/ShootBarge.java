package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootBargeConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.superstructure.Superstructure;

public class ShootBarge extends Command {
    private final EagleSwerveDrivetrain drivetrain;
    private final Superstructure superstructure;
    private boolean debug = false;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(4, 0, 0);
    
    private final NetworkTable autoAlignTable = NetworkTableInstance.getDefault().getTable("AutoAlign");
    private final StructPublisher<Pose2d> targetPose = autoAlignTable.getStructTopic("BargePose", Pose2d.struct).publish();
    private final DoublePublisher driveXVel = autoAlignTable.getDoubleTopic("Drive VelX").publish();

    public ShootBarge(EagleSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;
        addRequirements(drivetrain, superstructure);
    }

    public ShootBarge(EagleSwerveDrivetrain drivetrain, Superstructure superstructure, boolean debug) {
        this(drivetrain, superstructure);
        this.debug = debug;
    }

    @Override
    public void initialize() {
        double veloX;
        Rotation2d headingReference;
        double xPosition;

        if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            veloX = ShootBargeConstants.TARGET_SHOOT_SPEED;
            headingReference = new Rotation2d();
            xPosition = ShootBargeConstants.BARGE_X_POS - ShootBargeConstants.SHOOT_DISTANCE;
        } else {
            veloX = -ShootBargeConstants.TARGET_SHOOT_SPEED;
            headingReference = new Rotation2d(Math.PI);
            xPosition = ShootBargeConstants.BARGE_X_POS + ShootBargeConstants.SHOOT_DISTANCE;
        }

        targetPose.set(new Pose2d(xPosition, drivetrain.getState().Pose.getY(), headingReference));

        superstructure.moveL4().schedule(); // Start moving to L4 while driving
        drivetrain.setControl(driveRequest
                    .withVelocityX(veloX)
                    .withVelocityY(0)
                    .withTargetDirection(headingReference));
    }

    @Override
    public void execute() {
        driveXVel.set(drivetrain.getState().Speeds.vxMetersPerSecond);
    }
    
    @Override
    public boolean isFinished() {
        if(Utils.isSimulation() || debug) {
            return true;
        }

        boolean isAtBarge = Math.abs(drivetrain.getState().Pose.getX() - ShootBargeConstants.BARGE_X_POS) <= ShootBargeConstants.SHOOT_DISTANCE;
        boolean isAtSpeed = Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) >= ShootBargeConstants.TARGET_SHOOT_SPEED;

        return isAtBarge && isAtSpeed;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}