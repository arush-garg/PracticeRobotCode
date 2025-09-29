package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericRollerSubsystem extends SubsystemBase {
    protected final TalonFX m_motor;

    public GenericRollerSubsystem(int motorId, String canbus, boolean inverted) {
        m_motor = new TalonFX(motorId, canbus);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        m_motor.getConfigurator().apply(config);
    }

    public Command run(double voltage) {
        //System.out.println("" + voltage);
        return runOnce(() -> {
            m_motor.setVoltage(voltage);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            m_motor.setVoltage(0);
        });
    }
}
