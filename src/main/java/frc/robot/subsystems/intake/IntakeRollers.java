package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.GenericRollerSubsystem;

public class IntakeRollers extends GenericRollerSubsystem {
    public IntakeRollers() {
        super(IntakeConstants.Rollers.MOTOR_ID, "rio", true);
    }

    public Command setManualVoltage(double joystickPosition) {
		return run(
				() -> {
					m_motor.setVoltage(joystickPosition * IntakeConstants.Rollers.MAX_VOLTS
							/ IntakeConstants.Rollers.MANUAL_RATIO);
				});
	}
}
