package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.*;
import frc.robot.utils.GenericRollerSubsystem;

public class EndEffectorRollers extends GenericRollerSubsystem {
    public EndEffectorRollers() {
        super(EndEffectorConstants.Rollers.MOTOR_ID, "rio", false);
    }

    public void runFunc(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public boolean isStalled() {
        return Math.abs(m_motor.getStatorCurrent().getValueAsDouble()) > EndEffectorConstants.Rollers.STALL_CURRENT;
    }

    public Command holdAlgae() {
        return Commands.runOnce(() -> run(EndEffectorConstants.Rollers.RETAIN_ALGAE));
    }

    public Command stopHoldingAlgae() {
        return Commands.runOnce(() -> stop());
    }

    @Override
    public void periodic() {
    }
}
