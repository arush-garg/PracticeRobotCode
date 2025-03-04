// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.Drive.EagleSwerveTelemetry;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max

  // swerve
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
  public final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // controllers
  private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
  private final CommandXboxController m_operatorController = new CommandXboxController(2);
  private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

  // auto
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_leftJoystick.getY() * MaxSpeed)
            .withVelocityY(-m_leftJoystick.getX() * MaxSpeed)
            .withRotationalRate(-m_rightJoystick.getX() * MaxAngularRate)));

    // todo: sysid routines for tuning
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset yaw
    m_operatorController.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
