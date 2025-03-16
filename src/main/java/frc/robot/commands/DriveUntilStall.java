package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        drivetrain.setControl(drive
        .withSpeeds(new ChassisSpeeds(
                AutoAlignConstants.FORWARDS_SPEED,
                0.0,
                0.0)));
        System.out.println("executing drive til stall");
            var motor = drivetrain.getModule(0).getDriveMotor();
            System.out.println(motor.getStatorCurrent().getValueAsDouble());
            if (motor.getStatorCurrent().getValueAsDouble() >= AutoAlignConstants.CURRENT_THRESHOLD) {
                System.out.println("is stalled is true now");
                isStalled = true;
            }
    }

    @Override
    public void initialize() {
        System.out.println("running drive til stall");
    }

    @Override
    public boolean isFinished() {
        System.out.println("am i finished? " + isStalled);
        if (isStalled) {
            isStalled = false;
            return true;
        } else {
            return false;
        }
    }
}
