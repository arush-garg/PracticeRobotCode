package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.EndEffectorConstants;

public class EndEffectorRollers extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(EndEffectorConstants.Rollers.MOTOR_ID, "rio");

    public EndEffectorRollers() {
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setInverted(true);
    }

    private void run(double speed) {
        m_motor.setVoltage(speed);
    }

    private void stop() {
        m_motor.setVoltage(0);
    }
}
