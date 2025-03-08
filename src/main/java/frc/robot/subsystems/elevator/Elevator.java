// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase {

  private final TalonFX m_master = new TalonFX(ElevatorConstants.MASTER_MOTOR_ID, "rio");
  private final TalonFX m_slave = new TalonFX(ElevatorConstants.SLAVE_MOTOR_ID, "rio");
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final ElasticSender m_elastic;

  public Elevator(boolean debug) {
	m_elastic = new ElasticSender("Elevator", debug);
	m_elastic.addButton("Zero", zero());
	m_elastic.addButton("Kill", kill());

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotorOutputConfigs masterMotorConfigs = new MotorOutputConfigs();
    masterMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    masterMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    MotorOutputConfigs slaveMotorConfigs = new MotorOutputConfigs();
    slaveMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;
    fdb.FeedbackRotorOffset = ElevatorConstants.OFFSET;

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_CRUISE_VELOCITY;
    mm.MotionMagicAcceleration = ElevatorConstants.MOTION_ACCELERATION;
    mm.MotionMagicJerk = ElevatorConstants.MOTION_JERK;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kS = ElevatorConstants.kS;
    slot0.kG = ElevatorConstants.kG;
    slot0.kV = ElevatorConstants.kV;
    slot0.kA = ElevatorConstants.kA;
    slot0.kP = ElevatorConstants.kP;
    slot0.kI = ElevatorConstants.kI;
    slot0.kD = ElevatorConstants.kD;

    m_master.getConfigurator().apply(cfg);
    m_master.getConfigurator().apply(masterMotorConfigs);
    m_slave.getConfigurator().apply(cfg);
    m_slave.getConfigurator().apply(slaveMotorConfigs);

    m_slave.setControl(new Follower(ElevatorConstants.MASTER_MOTOR_ID, false));
  }

  public Command moveTo(double position) {
    return runOnce(
        () -> {
          m_master.setControl(m_mmReq.withPosition(position).withSlot(0));
        });
  }

  public Command zero() {
    return runOnce(
        () -> {
          m_master.setPosition(ElevatorConstants.OFFSET);
        });
  }

  public Command kill() {
    return runOnce(
        () -> {
          m_master.disable();
          m_slave.disable();
        });
  }

  public Command setManualVoltage(double joystickPosition) {
    return run(
        () -> {
          m_master.setVoltage(joystickPosition * ElevatorConstants.MAX_VOLTS / ElevatorConstants.MANUAL_RATIO);
        });
  }

  @Override
	public void periodic() {
		m_elastic.periodic();
	}
}
