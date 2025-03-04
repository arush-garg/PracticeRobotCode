package frc.robot.subsystems.endeffector;

import frc.robot.constants.*;
import frc.robot.utils.GenericRollerSubsystem;

public class EndEffectorRollers extends GenericRollerSubsystem {
    public EndEffectorRollers() {
        super(EndEffectorConstants.Rollers.MOTOR_ID, "rio", true);
    }
}