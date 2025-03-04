// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.EndEffectorConstants.WristPosition;
import frc.robot.utils.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorWrist extends SubsystemBase {

	private final TalonFX m_motor = new TalonFX(EndEffectorConstants.Wrist.MOTOR_ID, "rio");
	private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	private boolean debug;
	private ElasticSender m_elastic;
	private WristPosition m_position;

	public EndEffectorWrist(boolean debug) {
		this.debug = debug;
		m_elastic = new ElasticSender("EE: Wrist", debug);

		TalonFXConfiguration cfg = new TalonFXConfiguration();

		MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
		motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
		motorConfigs.NeutralMode = NeutralModeValue.Brake;

		FeedbackConfigs fdb = cfg.Feedback;
		fdb.SensorToMechanismRatio = EndEffectorConstants.Wrist.GEAR_RATIO;
		fdb.FeedbackRotorOffset = EndEffectorConstants.Wrist.OFFSET;

		MotionMagicConfigs mm = cfg.MotionMagic;
		mm.MotionMagicCruiseVelocity = EndEffectorConstants.Wrist.MOTION_CRUISE_VELOCITY;
		mm.MotionMagicAcceleration = EndEffectorConstants.Wrist.MOTION_ACCELERATION;
		mm.MotionMagicJerk = EndEffectorConstants.Wrist.MOTION_JERK;

		Slot0Configs slot0 = cfg.Slot0;
		slot0.GravityType = GravityTypeValue.Arm_Cosine;
		slot0.kS = EndEffectorConstants.Wrist.kS;
		slot0.kG = EndEffectorConstants.Wrist.kG;
		slot0.kV = EndEffectorConstants.Wrist.kV;
		slot0.kA = EndEffectorConstants.Wrist.kA;
		slot0.kP = EndEffectorConstants.Wrist.kP;
		slot0.kI = EndEffectorConstants.Wrist.kI;
		slot0.kD = EndEffectorConstants.Wrist.kD;

		m_motor.getConfigurator().apply(cfg);
	}

	public void ElasticInit() {

	}

	public Command moveTo(WristPosition position) {
		return runOnce(
				() -> {
					m_position = position;
					m_motor.setControl(m_mmReq.withPosition(position.getAngle()).withSlot(0));
				});
	}

	public Command moveToNextPosition() {
		return runOnce(
				() -> {
					WristPosition nextPosition = null;
					switch (m_position) {
						case L1_PRE_ANGLE:
							nextPosition = WristPosition.L1_SCORE_ANGLE;
							break;
						case L2_PRE_ANGLE:
							nextPosition = WristPosition.L2_SCORE_ANGLE;
							break;
						case L3_PRE_ANGLE:
							nextPosition = WristPosition.L3_SCORE_ANGLE;
							break;
						case L4_PRE_ANGLE:
							nextPosition = WristPosition.L4_SCORE_ANGLE;
							break;
						case DEALGAE_HIGH_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						case DEALGAE_LOW_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						case L1_SCORE_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						case L2_SCORE_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						case L3_SCORE_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						case L4_SCORE_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						case SCORE_BARGE_ANGLE:
							nextPosition = WristPosition.STOW_WRIST_ANGLE;
							break;
						default:
							nextPosition = m_position;
							break;
					}
					moveTo(nextPosition);
				});
	}

	public Command zero() {
		return runOnce(
				() -> {
					m_motor.setPosition(EndEffectorConstants.Wrist.OFFSET);
				});
	}

	public Command kill() {
		return runOnce(
				() -> {
					m_motor.disable();
				});
	}

	public Command setManualVoltage(double joystickPosition) {
		return run(
				() -> {
					m_motor.setVoltage(joystickPosition * EndEffectorConstants.Wrist.MAX_VOLTS
							/ EndEffectorConstants.Wrist.MANUAL_RATIO);
				});
	}

	@Override
	public void periodic() {
		if (debug) {
			m_elastic.update();
		}
	}
}
