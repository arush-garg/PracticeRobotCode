// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import frc.robot.constants.ElevatorConstants;
import frc.robot.utils.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.XboxController;

public class Elevator extends SubsystemBase {

  private final TalonFX m_fx = new TalonFX(ElevatorConstants.MOTOR_ID, "rio");
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  public boolean inManual = false;
  private XboxController xbox;

  public Elevator(XboxController controller) {
    this.xbox = controller;
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;
    fdb.FeedbackRotorOffset = ElevatorConstants.OFFSET;

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    Slot0Configs slot0 = cfg.Slot0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kS = ElevatorConstants.kS;
    slot0.kG = ElevatorConstants.kG;
    slot0.kV = ElevatorConstants.kV;
    slot0.kA = ElevatorConstants.kA;
    slot0.kP = ElevatorConstants.kP;
    slot0.kI = ElevatorConstants.kI;
    slot0.kD = ElevatorConstants.kD;

    m_fx.getConfigurator().apply(cfg);
  }

  public Command moveTo(double position) {
    return runOnce(
        () -> {
          m_fx.setControl(m_mmReq.withPosition(position).withSlot(0));
        });
  }

  public Command zero(double position) {
    return runOnce(
        () -> {
          m_fx.setPosition(ElevatorConstants.OFFSET);
        });
  }

  @Override
  public void periodic() {
    if (xbox.getXButton() && !Utils.NearZero(xbox.getLeftY())) {
      inManual = true;
    }

    if (xbox.getXButtonReleased() || Utils.NearZero(xbox.getLeftY())) {
      inManual = false;
    }

    if (inManual) {
      m_fx.setVoltage(xbox.getLeftY() * ElevatorConstants.MAX_VOLTS / ElevatorConstants.MANUAL_RATIO);
    } else if (m_fx.getPosition().getValueAsDouble() < 0.01) {
      m_fx.disable();
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
