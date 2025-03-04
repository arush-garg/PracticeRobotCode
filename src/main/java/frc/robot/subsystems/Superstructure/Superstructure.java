package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.EndEffectorConstants.WristPosition;
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

    public Command intake() {
        return Commands.sequence(
                m_elevator.moveTo(ElevatorConstants.INTAKE_UP_HEIGHT),
                m_eeWrist.moveTo(WristPosition.INTAKE),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
                m_intakeRollers.run(IntakeConstants.Rollers.INTAKE_CORAL_VOLTS),
                m_elevator.moveTo(0),
                m_eeRollers.run(EndEffectorConstants.Rollers.INTAKE_CORAL_VOLTS),
                m_intakeRollers.stop(),
                m_eeRollers.stop());
    }

    public Command moveL1() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L1_HEIGHT),
                m_eeWrist.moveTo(
                        gpMode == GPMode.Coral ? WristPosition.L1_PRE_ANGLE : WristPosition.SCORE_PROCESSOR_ANGLE));
    }

    public Command moveL2() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L2_HEIGHT),
                m_eeWrist
                        .moveTo(gpMode == GPMode.Coral ? WristPosition.L2_PRE_ANGLE : WristPosition.DEALGAE_LOW_ANGLE));
    }

    public Command moveL3() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L3_HEIGHT),
                m_eeWrist.moveTo(
                        gpMode == GPMode.Coral ? WristPosition.L3_PRE_ANGLE : WristPosition.DEALGAE_HIGH_ANGLE));
    }

    public Command moveL4() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L4_HEIGHT),
                m_eeWrist
                        .moveTo(gpMode == GPMode.Coral ? WristPosition.L4_PRE_ANGLE : WristPosition.SCORE_BARGE_ANGLE));
    }
}
