// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.EndEffectorWrist;

public class RobotContainer {
  private final CommandJoystick m_leftJoystick = new CommandJoystick(0);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
  private final CommandXboxController m_operatorController = new CommandXboxController(2);
  private final CommandXboxController m_buttonBoard = new CommandXboxController(3);

  private final Elevator m_elevator = new Elevator();
  private final EndEffectorWrist m_wrist = new EndEffectorWrist(false);

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
    m_operatorController.leftTrigger().and(m_operatorController.x()).onTrue(m_wrist.zero());
  }

  // public Command getAutonomousCommand() {
  // }
}
