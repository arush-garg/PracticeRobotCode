package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.*;
import frc.robot.utils.GenericRollerSubsystem;

public class EndEffectorRollers extends GenericRollerSubsystem {
    private boolean holdingAlgaeVoltage = false;

    public EndEffectorRollers() {
        super(EndEffectorConstants.Rollers.MOTOR_ID, "rio", true);
    }

    public boolean isStalled() {
        return m_motor.getStatorCurrent().getValueAsDouble() > EndEffectorConstants.Rollers.STALL_CURRENT;
    }

    public Command holdAlgae() {
        return Commands.runOnce(() -> holdingAlgaeVoltage = true);
    }

    public Command stopHoldingAlgae() {
        return Commands.runOnce(() -> holdingAlgaeVoltage = false);
    }

    @Override
    public void periodic() {
        if (holdingAlgaeVoltage) {
            m_motor.setVoltage(EndEffectorConstants.Rollers.RETAIN_ALGAE);
        }
    }
}
