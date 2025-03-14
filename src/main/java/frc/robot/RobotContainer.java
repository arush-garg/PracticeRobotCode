// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.channel.Channel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorRollers;
import frc.robot.subsystems.endeffector.EndEffectorWrist;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
  private final CommandXboxController m_operatorController = new CommandXboxController(2);
  private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

  private final Elevator m_elevator = new Elevator(true);
  private final EndEffectorWrist m_eeWrist = new EndEffectorWrist(true);
  private final EndEffectorRollers m_eeRollers = new EndEffectorRollers();
  private final IntakeWrist m_intakeWrist = new IntakeWrist(true);
  private final IntakeRollers m_intakeRollers = new IntakeRollers();
  private final Channel m_channel = new Channel(true);
  private final Superstructure m_superstructure = new Superstructure(m_elevator, m_eeWrist, m_eeRollers, m_intakeWrist,
      m_intakeRollers, m_channel, true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // manual commands
    m_operatorController.x()
        .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
        .whileTrue(m_elevator.setManualVoltage(m_operatorController.getLeftY()));
    m_operatorController.b()
        .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
        .whileTrue(m_intakeWrist.setManualVoltage(m_operatorController.getLeftY()));
    m_operatorController.y().and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
        .whileTrue(m_eeWrist.setManualVoltage(m_operatorController.getLeftY()));

    // zero commands
    m_operatorController.button(7).and(m_operatorController.x())
        .onTrue(m_elevator.zero().ignoringDisable(true));
    m_operatorController.button(7).and(m_operatorController.y())
        .onTrue(m_eeWrist.zero().ignoringDisable(true));
    m_operatorController.button(7).and(m_operatorController.b())
        .onTrue(m_intakeWrist.zero().ignoringDisable(true));

    // gpMode switching
    m_buttonBoard.button(5).onTrue(
        m_superstructure.switchMode());

    // scoring commands
    m_rightJoystick.trigger().onTrue(m_superstructure.intake());
    m_leftJoystick.trigger().onTrue(m_superstructure.score());
    m_buttonBoard.button(1).onTrue(m_superstructure.moveL1());
    // m_buttonBoard.button(2).onTrue(m_superstructure.moveL2());
    // m_buttonBoard.button(3).onTrue(m_superstructure.moveL3());
    // m_buttonBoard.button(4).onTrue(m_superstructure.moveL4());

    // stow commands
    m_leftJoystick.button(2).onTrue(m_superstructure.stow());
    m_operatorController.povUp().onTrue(m_superstructure.stow());
  }

  // public void zeroingButtons() {
  // //
  // m_operatorController.button(7).and(m_operatorController.b()).onTrue(m_intakeWrist.zero());
  // if
  // (m_operatorController.button(7).and(m_operatorController.b()).getAsBoolean())
  // {
  // m_intakeWrist.zeroing();
  // }
  // }

  // public Command getAutonomousCommand() {
  // }
}
