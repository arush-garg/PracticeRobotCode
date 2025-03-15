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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveToPoseCommand.TargetPosition;
import frc.robot.commands.DriveUntilStall;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.Drive.EagleSwerveTelemetry;
import frc.robot.subsystems.channel.Channel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorRollers;
import frc.robot.subsystems.endeffector.EndEffectorWrist;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.superstructure.GPMode;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.vision.Vision;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max

        // swerve
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
        public final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Vision vision = new Vision(drivetrain);

        private boolean slowModeOn = false;

        // controllers
        private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
        private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
        private final CommandXboxController m_operatorController = new CommandXboxController(2);
        private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

        // auto
        private final SendableChooser<Command> autoChooser;

        private final Elevator m_elevator = new Elevator(true);
        private final EndEffectorWrist m_eeWrist = new EndEffectorWrist(true);
        private final EndEffectorRollers m_eeRollers = new EndEffectorRollers();
        private final IntakeWrist m_intakeWrist = new IntakeWrist(true);
        private final IntakeRollers m_intakeRollers = new IntakeRollers();
        private final Channel m_channel = new Channel(true);
        private final Superstructure m_superstructure = new Superstructure(m_elevator, m_eeWrist, m_eeRollers,
                        m_intakeWrist,
                        m_intakeRollers, m_channel, true);

        public RobotContainer() {
                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(
                                                                -m_leftJoystick.getY() * MaxSpeed * (slowModeOn
                                                                                ? DriveConstants.SLOW_MODE_MULT
                                                                                : 1))
                                                .withVelocityY(
                                                                -m_leftJoystick.getX() * MaxSpeed * (slowModeOn
                                                                                ? DriveConstants.SLOW_MODE_MULT
                                                                                : 1))
                                                .withRotationalRate(
                                                                -m_rightJoystick.getX() * MaxAngularRate
                                                                                * (slowModeOn ? DriveConstants.SLOW_MODE_MULT
                                                                                                : 1))));

                m_rightJoystick.button(2)
                                .onTrue(Commands.runOnce(() -> slowModeOn = true))
                                .onFalse(Commands.runOnce(() -> slowModeOn = false));

                // sysid routines for tuning

                // m_operatorController.button(7).and(m_operatorController.y())
                // .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // m_operatorController.button(7).and(m_operatorController.x())
                // .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // m_operatorController.button(8).and(m_operatorController.y())
                // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // m_operatorController.button(8).and(m_operatorController.x())
                // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset yaw
                m_operatorController.button(8).onTrue(drivetrain.runOnce(() -> {
                        System.out.println("Zeroing");
                        drivetrain.seedFieldCentric();
                        drivetrain.resetTranslation(new Translation2d(0, 0));
                }));

                m_buttonBoard.button(11).whileTrue(new DriveToPoseCommand(drivetrain, vision, TargetPosition.RIGHT));
                m_buttonBoard.button(12).whileTrue(new DriveToPoseCommand(drivetrain, vision, TargetPosition.LEFT));
                m_buttonBoard.button(13).whileTrue(new DriveToPoseCommand(drivetrain, vision, TargetPosition.ALGAE));

                // testing drive until stall
                m_buttonBoard.button(14).whileTrue(new DriveUntilStall(drivetrain));

                drivetrain.registerTelemetry(logger::telemeterize);

                // manual commands
                m_operatorController.x()
                                .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.05))
                                .whileTrue(m_elevator.setManualVoltage(m_operatorController.getLeftY()));
                m_operatorController.b()
                                .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.05))
                                .whileTrue(m_intakeWrist.setManualVoltage(m_operatorController.getLeftY()));
                m_operatorController.y().and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.05))
                                .whileTrue(m_eeWrist.setManualVoltage(m_operatorController.getLeftY()));

                m_operatorController.b()
                                .and(m_operatorController.leftBumper())
                                .onTrue(m_superstructure.ejectIntake());
                
                m_operatorController.y()
                                .and(m_operatorController.leftBumper())
                                .onTrue(m_superstructure.ejectEE());

                // zero commands
                m_operatorController.button(7).and(m_operatorController.x())
                                .onTrue(m_elevator.zero().ignoringDisable(true));
                m_operatorController.button(7).and(m_operatorController.y())
                                .onTrue(m_eeWrist.zero().ignoringDisable(true));
                m_operatorController.button(7).and(m_operatorController.b())
                                .onTrue(m_intakeWrist.zero().ignoringDisable(true));

                // gpMode switching
                m_buttonBoard.button(5).onTrue(m_superstructure.switchMode());

                // scoring commands
                m_rightJoystick.trigger().onTrue(m_superstructure.intake());
                m_leftJoystick.trigger().onTrue(m_superstructure.score());
                m_buttonBoard.button(1).onTrue(m_superstructure.moveL1());
                m_buttonBoard.button(2).onTrue(m_superstructure.moveL2());
                m_buttonBoard.button(3).onTrue(m_superstructure.moveL3());
                m_buttonBoard.button(4).onTrue(m_superstructure.moveL4());

                // stow commands
                m_leftJoystick.button(2).onTrue(m_superstructure.stow());
                m_operatorController.povUp().onTrue(m_superstructure.stow());
        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }

        public void resetMechs() {
			System.out.println("reseting mechs") ; 
			m_intakeRollers.stop();
			m_intakeWrist.kill();
			m_channel.stop();
			m_elevator.kill();
			m_eeWrist.kill();
			m_eeRollers.stop();
			/*if (m_superstructure.getGPMode() == GPMode.Algae) {
				m_superstructure.switchMode();
            }*/
        }

}
