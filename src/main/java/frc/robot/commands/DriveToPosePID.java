package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveToPoseConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class DriveToPosePID extends Command {

    private final EagleSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> target;

    private final ProfiledPIDController driveController = new ProfiledPIDController(
            DriveToPoseConstants.DRIVE_PID.kP, DriveToPoseConstants.DRIVE_PID.kI,
            DriveToPoseConstants.DRIVE_PID.kD,
            new TrapezoidProfile.Constraints(DriveToPoseConstants.DRIVE_MAX_VELOCITY.in(MetersPerSecond),
                    DriveToPoseConstants.DRIVE_MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            DriveToPoseConstants.THETA_PID.kP, DriveToPoseConstants.THETA_PID.kI,
            DriveToPoseConstants.THETA_PID.kD,
            new TrapezoidProfile.Constraints(DriveToPoseConstants.THETA_MAX_VELOCITY.in(DegreesPerSecond),
                    DriveToPoseConstants.THETA_MAX_ACCELERATION.in(DegreesPerSecondPerSecond)));

    private Translation2d lastSetpointTranslation = new Translation2d();
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    private boolean running = false;

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    public DriveToPosePID(EagleSwerveDrivetrain drivetrain, Supplier<Pose2d> target) {
        this.drivetrain = drivetrain;
        this.target = target;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getState().Pose;
        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds,
                currentPose.getRotation());
        Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
                fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                currentPose.getTranslation().getDistance(target.get().getTranslation()),
                Math.min(
                        0.0,
                        -linearFieldVelocity
                                .rotateBy(
                                        target
                                                .get()
                                                .getTranslation()
                                                .minus(currentPose
                                                        .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        running = true;
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d targetPose = target.get();

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - DriveToPoseConstants.FF_MIN_RADIUS.in(Meters))
                        / (DriveToPoseConstants.FF_MAX_RADIUS.in(Meters)
                                - DriveToPoseConstants.FF_MIN_RADIUS.in(Meters)),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);

        double calc = driveController.calculate(driveErrorAbs, 0.0);

        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + calc;
        if (currentDistance < DriveToPoseConstants.DRIVE_TOLERANCE.in(Units.Meter))
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0,
                        new Rotation2d()))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < DriveToPoseConstants.THETA_TOLERANCE.in(Radian))
            thetaVelocity = 0.0;

        Translation2d driveVelocity = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
                .getTranslation();

        ChassisSpeeds requestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());

        drivetrain.setControl(driveRequest.withVelocityX(requestedSpeeds.vxMetersPerSecond)
                .withVelocityY(requestedSpeeds.vyMetersPerSecond)
                .withRotationalRate(requestedSpeeds.omegaRadiansPerSecond));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        running = false;
    }

    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    public boolean withinTolerance(Distance driveTolerance, Angle thetaTolerance) {
        return running
                && Math.abs(driveErrorAbs) < driveTolerance.in(Units.Meter)
                && Math.abs(thetaErrorAbs) < thetaTolerance.in(Units.Radian);
    }

    @Override
    public boolean isFinished() {
        return withinTolerance(DriveToPoseConstants.DRIVE_TOLERANCE, DriveToPoseConstants.THETA_TOLERANCE);
    }
}