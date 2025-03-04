package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericRollerSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    public GenericRollerSubsystem(int motorId, String canbus, boolean inverted) {
        m_motor = new TalonFX(motorId, canbus);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        m_motor.getConfigurator().apply(config);
    }

    private void run(double speed) {
        m_motor.setVoltage(speed);
    }

    private void stop() {
        m_motor.setVoltage(0);
    }
}
