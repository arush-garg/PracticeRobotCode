package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.utils.GenericRollerSubsystem;

public class IntakeRollers extends GenericRollerSubsystem {
    public IntakeRollers() {
        super(IntakeConstants.Rollers.MOTOR_ID, "rio", true);
    }
}
