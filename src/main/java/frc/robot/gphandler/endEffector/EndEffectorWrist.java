// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gphandler.endEffector;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import frc.robot.utils.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.XboxController;


public class EndEffectorWrist extends SubsystemBase {

	private final TalonFX m_fx = new TalonFX(EndEffectorConstants.Wrist.MOTOR_ID, "rio");
	private final TalonFXConfiguration cfg = new TalonFXConfiguration();
	private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	private boolean inManual = false;
	private boolean elasticOn;
	private XboxController xbox;
	private ElasticSender m_elastic;

  	public EndEffectorWrist(XboxController controller, boolean debug) {
		this.xbox = controller; 
		this.elasticOn = debug;
		m_elastic = new ElasticSender("EE: Wrist", elasticOn);
		
		/* Configure gear ratio */
		FeedbackConfigs fdb = cfg.Feedback;
		fdb.FeedbackRemoteSensorID = EndEffectorConstants.Wrist.ENCODER_ID;
		fdb.SensorToMechanismRatio = EndEffectorConstants.Wrist.GEAR_RATIO; // 12.8 rotor rotations per mechanism rotation
		fdb.FeedbackRotorOffset = EndEffectorConstants.Wrist.OFFSET;

		/* Configure Motion Magic */
		MotionMagicConfigs mm = cfg.MotionMagic;
		mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
		.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
		// Take approximately 0.1 seconds to reach max accel 
		.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

		Slot0Configs slot0 = cfg.Slot0;
		slot0.GravityType = GravityTypeValue.Arm_Cosine;
		slot0.kS = EndEffectorConstants.Wrist.kS; // Add 0.25 V output to overcome static friction
		slot0.kG = EndEffectorConstants.Wrist.kG; // Add 0.25 V output to overcome static friction
		slot0.kV = EndEffectorConstants.Wrist.kV; // A velocity target of 1 rps results in 0.12 V output
		slot0.kA = EndEffectorConstants.Wrist.kA;; // An acceleration of 1 rps/s requires 0.01 V output
		slot0.kP = EndEffectorConstants.Wrist.kP;; // A position error of 0.2 rotations results in 12 V output
		slot0.kI = EndEffectorConstants.Wrist.kI;; // No output for integrated error
		slot0.kD = EndEffectorConstants.Wrist.kS;; // A velocity error of 1 rps results in 0.5 V output

		StatusCode status = StatusCode.StatusCodeNotInitialized;
		for (int i = 0; i < 5; ++i) {
		status = m_fx.getConfigurator().apply(cfg);
		if (status.isOK()) break;
		}
		if (!status.isOK()) {
		System.out.println("Could not configure device. Error: " + status.toString());
		}
	
  	}

	public void ElasticInit() {

	}


	public Command moveTo(double position) {
		return runOnce(
			() -> {
				m_fx.setControl(m_mmReq.withPosition(position).withSlot(0));
			});
	}

	public Command kill(double position) {
		return runOnce(
		() -> {
			m_fx.disable();
		});
	}

  	@Override
	public void periodic() {
		if (elasticOn) {
			m_elastic.update();
		}

		if (xbox.getYButton() && !Utils.NearZero(xbox.getLeftY())) {
			inManual = true;
		} 

		if (xbox.getYButtonReleased() || Utils.NearZero(xbox.getLeftY())) {
			inManual = false;
		}

		if (inManual) {
			m_fx.setVoltage(xbox.getLeftY() * ElevatorConstants.MAX_VOLTS / ElevatorConstants.MANUAL_RATIO);
		}
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
