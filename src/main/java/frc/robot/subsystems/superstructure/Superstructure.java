package frc.robot.subsystems.superstructure;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.constants.*;
import frc.robot.subsystems.channel.Channel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorRollers;
import frc.robot.subsystems.endeffector.EndEffectorWrist;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;

public class Superstructure {
    private GPMode gpMode = GPMode.Coral;

    private final Elevator m_elevator;
    private final EndEffectorWrist m_eeWrist;
    private final EndEffectorRollers m_eeRollers;
    private final IntakeWrist m_intakeWrist;
    private final IntakeRollers m_intakeRollers;
    private final Channel m_channel;

    public Superstructure(Elevator elevator, EndEffectorWrist eeWrist, EndEffectorRollers eeRollers,
            IntakeWrist intakeWrist, IntakeRollers intakeRollers, Channel channel) {
        this.m_elevator = elevator;
        this.m_eeWrist = eeWrist;
        this.m_eeRollers = eeRollers;
        this.m_intakeWrist = intakeWrist;
        this.m_intakeRollers = intakeRollers;
        this.m_channel = channel;
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

    public GPMode getGPMode() {
        return gpMode;
    }

    public Command intake() {
        return Commands.sequence(
                m_eeWrist.moveTo(EndEffectorWristPosition.INTAKE_ANGLE),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
                m_intakeRollers.run(IntakeConstants.Rollers.INTAKE_CORAL_VOLTS),
                Commands.waitUntil(m_channel.coralInEndEffectorSupplier),
                Commands.parallel(m_intakeRollers.stop(), m_channel.stop(), m_eeRollers.stop()));
    }

    public Command moveL1() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L1_HEIGHT),
                m_eeWrist.moveTo(
                        gpMode == GPMode.Coral ? EndEffectorWristPosition.L1_PRE_ANGLE
                                : EndEffectorWristPosition.SCORE_PROCESSOR_ANGLE));
    }

    public Command moveL2() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L2_HEIGHT),
                m_eeWrist
                        .moveTo(gpMode == GPMode.Coral ? EndEffectorWristPosition.L2_PRE_ANGLE
                                : EndEffectorWristPosition.DEALGAE_LOW_ANGLE));
    }

    public Command moveL3() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L3_HEIGHT),
                m_eeWrist.moveTo(
                        gpMode == GPMode.Coral ? EndEffectorWristPosition.L3_PRE_ANGLE
                                : EndEffectorWristPosition.DEALGAE_HIGH_ANGLE));
    }

    public Command moveL4() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.L4_HEIGHT),
                m_eeWrist
                        .moveTo(gpMode == GPMode.Coral ? EndEffectorWristPosition.L4_PRE_ANGLE
                                : EndEffectorWristPosition.SCORE_BARGE_ANGLE));
    }

    public Command score() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, m_eeWrist.moveToNextPosition()),
                        Map.entry(GPMode.Algae, m_eeRollers.run(4))),
                this::getGPMode);
    }

    public Command stow() {
        return Commands.parallel(
                m_elevator.moveTo(ElevatorConstants.STOWED_HEIGHT),
                m_eeWrist.moveTo(EndEffectorWristPosition.STOW_WRIST_ANGLE));
    }
}
