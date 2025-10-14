package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootBargeConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.superstructure.Superstructure;

public class ShootBarge extends Command {
    private final EagleSwerveDrivetrain drivetrain;
    private final Superstructure superstructure;

    private final SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);
    
    private ChassisSpeeds targetSpeeds;

    public ShootBarge(EagleSwerveDrivetrain drivetrain, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;
        addRequirements(drivetrain, superstructure);
    }

    @Override
    public void initialize() {
        if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            targetSpeeds = new ChassisSpeeds(ShootBargeConstants.TARGET_SHOOT_SPEED, 0, 0);
        } else {
            targetSpeeds = new ChassisSpeeds(-ShootBargeConstants.TARGET_SHOOT_SPEED, 0, 0);
        }
        superstructure.moveL4().schedule(); // Start moving to L4 while driving
        drivetrain.setControl(drive.withSpeeds(targetSpeeds));
    }

    @Override
    public void execute() {
    }
    
    @Override
    public boolean isFinished() {
        // TODO: Define conditions for finishing the command (at target position and speed)
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            superstructure.scoreAlgae();
        }
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}