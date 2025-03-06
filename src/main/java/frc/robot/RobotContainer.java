// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveToPoseCommand.TargetPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.Drive.EagleSwerveTelemetry;
import frc.robot.vision.Vision;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max

  // swerve
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
  public final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Vision vision = new Vision(drivetrain);

  // controllers
  private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
  private final CommandXboxController m_operatorController = new CommandXboxController(2);
  private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

  // auto
  //private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //autoChooser = AutoBuilder.buildAutoChooser("Tests");
    //SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_leftJoystick.getY() * MaxSpeed)
            .withVelocityY(-m_leftJoystick.getX() * MaxSpeed)
            .withRotationalRate(-m_rightJoystick.getX() * MaxAngularRate)));

    // sysid routines for tuning

    
    m_operatorController.button(7).and(m_operatorController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_operatorController.button(7).and(m_operatorController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_operatorController.button(8).and(m_operatorController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_operatorController.button(8).and(m_operatorController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset yaw
    //m_operatorController.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    m_buttonBoard.button(11).whileTrue(new DriveToPoseCommand(drivetrain, vision, TargetPosition.RIGHT));
    m_buttonBoard.button(12).whileTrue(new DriveToPoseCommand(drivetrain, vision, TargetPosition.LEFT));
    m_buttonBoard.button(13).whileTrue(new DriveToPoseCommand(drivetrain, vision, TargetPosition.ALGAE));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    return null;
  }
}
