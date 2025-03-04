package frc.robot.subsystems.EndEffector;

import frc.robot.constants.EndEffectorConstants;
import frc.robot.utils.GenericRollerSubsystem;

public class EndEffectorRollers extends GenericRollerSubsystem {
    public EndEffectorRollers() {
        super(EndEffectorConstants.Rollers.MOTOR_ID, "rio", true);
    }
}