package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.EndEffectorRollers;
import frc.robot.subsystems.EndEffector.EndEffectorWrist;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.Intake.IntakeWrist;

public class Superstructure {
    private GPMode gpMode = GPMode.Coral;

    private final Elevator m_elevator;
    private final EndEffectorWrist m_eeWrist;
    private final EndEffectorRollers m_eeRollers;
    private final IntakeWrist m_intakeWrist;
    private final IntakeRollers m_intakeRollers;

    public Superstructure(Elevator m_elevator, EndEffectorWrist m_eeWrist, EndEffectorRollers m_eeRollers,
            IntakeWrist m_intakeWrist, IntakeRollers m_intakeRollers) {
        this.m_elevator = m_elevator;
        this.m_eeWrist = m_eeWrist;
        this.m_eeRollers = m_eeRollers;
        this.m_intakeWrist = m_intakeWrist;
        this.m_intakeRollers = m_intakeRollers;
    }

    public Command switchMode() {
        return Commands.runOnce(() -> {
            if (gpMode == GPMode.Coral) {
                gpMode = GPMode.Algae;
            } else {
                gpMode = GPMode.Coral;
            }
        });
    }
}
