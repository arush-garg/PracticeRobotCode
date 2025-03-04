// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private final Elevator m_elevator = new Elevator();
  private final EndEffectorWrist m_eeWrist = new EndEffectorWrist(false);
  private final EndEffectorRollers m_eeRollers = new EndEffectorRollers();
  private final IntakeWrist m_intakeWrist = new IntakeWrist();
  private final IntakeRollers m_intakeRollers = new IntakeRollers();
  private final Channel m_channel = new Channel();
  private final Superstructure m_superstructure = new Superstructure(m_elevator, m_eeWrist, m_eeRollers, m_intakeWrist,
      m_intakeRollers, m_channel);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // elevator manual
    m_operatorController.x()
        .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
        .whileTrue(m_elevator.setManualVoltage(m_operatorController.getLeftY()));
    // elevator zero
    m_operatorController.leftTrigger().and(m_operatorController.x()).onTrue(m_elevator.zero());

    // wrist manual
    m_operatorController.y()
        .and(m_operatorController.axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01))
        .whileTrue(m_elevator.setManualVoltage(m_operatorController.getLeftY()));
    // wrist zero
    m_operatorController.leftTrigger().and(m_operatorController.x()).onTrue(m_eeWrist.zero());

    // gpMode switching
    m_buttonBoard.button(5).onTrue(
        m_superstructure.switchMode());

    // scoring commands
    m_rightJoystick.trigger().onTrue(m_superstructure.intake());
    m_leftJoystick.trigger().onTrue(m_superstructure.score());
    m_buttonBoard.button(0).onTrue(m_superstructure.moveL1());
    m_buttonBoard.button(1).onTrue(m_superstructure.moveL2());
    m_buttonBoard.button(2).onTrue(m_superstructure.moveL3());
    m_buttonBoard.button(3).onTrue(m_superstructure.moveL4());

    // stow commands
    m_leftJoystick.button(2).onTrue(m_superstructure.stow());
    // todo: oprator controller stow button
  }

  // public Command getAutonomousCommand() {
  // }
}
