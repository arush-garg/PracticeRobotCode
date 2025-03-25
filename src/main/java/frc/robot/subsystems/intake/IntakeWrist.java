// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElasticSender.ElasticSender;
import frc.robot.constants.IntakeConstants;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

public class IntakeWrist extends SubsystemBase {

    private final TalonFX m_motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, "rio");
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    private final ElasticSender m_sender;
    // For tuning
    private double m_targetPos;

    public IntakeWrist(boolean debug) {
        m_sender = new ElasticSender("Intake Wrist", debug);
        // m_sender.put("Pos Target", m_targetPos, true);
        // m_sender.addButton("Move to target", moveTo(m_targetPos));
        // m_sender.addButton("Zero", zero());
        // m_sender.addButton("Kill", kill());

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
        m_motor.getConfigurator().apply(motorConfigs);
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
                    System.out.println("Zeroing Intake");
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

    @Override
    public void periodic() {
        // m_targetPos = m_sender.getNumber("Pos Target");
    }

    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            IntakeConstants.Wrist.GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(0.43, 6.18),
            0.43,
            -Double.MAX_VALUE,
            Double.MAX_VALUE,
            false,
            0);

    @AutoLogOutput
    public Pose3d getPose() {
        return new Pose3d(new Translation3d(0, -0.35, 0.3),
                new Rotation3d(m_motor.getPosition().getValueAsDouble(), 0, 0));
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = m_motor.getSimState();
        talonFXSim.Orientation = ChassisReference.Clockwise_Positive;
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();
        m_armSim.setInputVoltage(motorVoltage.in(Volts));
        m_armSim.update(0.02);
        talonFXSim.setRawRotorPosition((m_armSim.getAngleRads() + IntakeConstants.Wrist.OFFSET) *
                IntakeConstants.Wrist.GEAR_RATIO / (2 * Math.PI));
        talonFXSim.setRotorVelocity(m_armSim.getVelocityRadPerSec() *
                IntakeConstants.Wrist.GEAR_RATIO / (2 * Math.PI));
    }
}
