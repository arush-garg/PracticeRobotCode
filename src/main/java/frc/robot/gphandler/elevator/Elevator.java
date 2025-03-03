// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gphandler.elevator;

import frc.robot.Constants.ElevatorConstants;

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


public class Elevator extends SubsystemBase {

	private final TalonFX m_fx = new TalonFX(ElevatorConstants.MOTOR_ID, "rio");
	private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	public boolean inManual = false;
	private XboxController xbox;

  public Elevator(XboxController controller) {
	this.xbox = controller; 
	TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO; // 12.8 rotor rotations per mechanism rotation
	fdb.FeedbackRotorOffset = ElevatorConstants.OFFSET;

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    Slot0Configs slot0 = cfg.Slot0;
	slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kS = ElevatorConstants.kS; // Add 0.25 V output to overcome static friction
    slot0.kG = ElevatorConstants.kG; // Add 0.25 V output to overcome static friction
    slot0.kV = ElevatorConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = ElevatorConstants.kA;; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = ElevatorConstants.kP;; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = ElevatorConstants.kI;; // No output for integrated error
    slot0.kD = ElevatorConstants.kS;; // A velocity error of 1 rps results in 0.5 V output

	StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
	
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command moveTo(double position) {
	// Inline construction of command goes here.
	// Subsystem::RunOnce implicitly requires `this` subsystem.
	return runOnce(
		() -> {
			m_fx.setControl(m_mmReq.withPosition(position).withSlot(0));
		});
  }

  public Command zero(double position) {
	// Inline construction of command goes here.
	// Subsystem::RunOnce implicitly requires `this` subsystem.
	return runOnce(
		() -> {
			m_fx.setPosition(ElevatorConstants.OFFSET);
		});
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

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
	}
	else if (m_fx.getPosition().getValueAsDouble() < 0.01) {
		m_fx.disable();
	}
  }

  @Override
  public void simulationPeriodic() {
	// This method will be called once per scheduler run during simulation
  }
}
