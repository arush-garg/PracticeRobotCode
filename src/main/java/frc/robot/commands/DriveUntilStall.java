package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class DriveUntilStall extends Command {
    private final EagleSwerveDrivetrain drivetrain;

    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean isStalled = false;

    public DriveUntilStall(
            EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))).schedule();
    }

    @Override
    public void execute() {
        drivetrain.applyRequest(() -> drive
                .withSpeeds(new ChassisSpeeds(
                        0.0,
                        AutoAlignConstants.FORWARDS_SPEED,
                        0.0)))
                .schedule();
        var motor = drivetrain.getModule(0).getDriveMotor();
        if (Math.abs(motor.getVelocity().getValueAsDouble()) <= AutoAlignConstants.VEL_THRESHOLD
                && motor.getStatorCurrent().getValueAsDouble() >= AutoAlignConstants.CURRENT_THRESHOLD) {
            isStalled = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isStalled;
    }
}
