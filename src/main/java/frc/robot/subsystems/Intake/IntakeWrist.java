// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeWrist extends SubsystemBase {

    private final TalonFX m_motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, "rio");
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public IntakeWrist() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = IntakeConstants.Wrist.GEAR_RATIO;
        fdb.FeedbackRotorOffset = IntakeConstants.Wrist.OFFSET;

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = IntakeConstants.Wrist.MOTION_CRUISE_VELOCITY;
        mm.MotionMagicAcceleration = IntakeConstants.Wrist.MOTION_ACCELERATION;
        mm.MotionMagicJerk = IntakeConstants.Wrist.MOTION_JERK;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.kS = IntakeConstants.Wrist.kS;
        slot0.kG = IntakeConstants.Wrist.kG;
        slot0.kV = IntakeConstants.Wrist.kV;
        slot0.kA = IntakeConstants.Wrist.kA;
        slot0.kP = IntakeConstants.Wrist.kP;
        slot0.kI = IntakeConstants.Wrist.kI;
        slot0.kD = IntakeConstants.Wrist.kD;

        m_motor.getConfigurator().apply(cfg);
    }

    public Command moveTo(double position) {
        return runOnce(
                () -> {
                    m_motor.setControl(m_mmReq.withPosition(position).withSlot(0));
                });
    }

    public Command zero() {
        return runOnce(
                () -> {
                    m_motor.setPosition(IntakeConstants.Wrist.OFFSET);
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
                    m_motor.setVoltage(joystickPosition * IntakeConstants.Wrist.MAX_VOLTS
                            / IntakeConstants.Wrist.MANUAL_RATIO);
                });
    }
}
