package frc.robot.subsystems.superstructure;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.ElasticSender.ElasticSender;
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
    private final ElasticSender m_elastic;

    public Superstructure(Elevator elevator, EndEffectorWrist eeWrist, EndEffectorRollers eeRollers,
            IntakeWrist intakeWrist, IntakeRollers intakeRollers, Channel channel, boolean debug) {
        this.m_elevator = elevator;
        this.m_eeWrist = eeWrist;
        this.m_eeRollers = eeRollers;
        this.m_intakeWrist = intakeWrist;
        this.m_intakeRollers = intakeRollers;
        this.m_channel = channel;

        m_elastic = new ElasticSender("Superstructure", debug);
        m_elastic.addButton("Switch Mode", switchMode());
        m_elastic.addButton("Intake", intake());
        m_elastic.addButton("Score", score());
        // m_elastic.addButton("Move L1", moveL1());
        // m_elastic.addButton("Move L2", moveL2());
        // m_elastic.addButton("Move L3", moveL3());
        // m_elastic.addButton("Move L4", moveL4());
        m_elastic.addButton("Stow", stow());
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
        System.out.println(gpMode);
        if (gpMode == null)
            return GPMode.Coral;
        return gpMode;
    }

    public Command intakeCoral() {
        return Commands.sequence(
                // m_eeWrist.moveTo(EndEffectorWristPosition.INTAKE_CORAL_ANGLE),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION),
                m_intakeRollers.run(IntakeConstants.Rollers.INTAKE_CORAL_VOLTS),
                m_channel.run(ChannelConstants.CHANNEL_VOLTS),
                Commands.waitUntil(m_channel.coralInEndEffectorSupplier),
                Commands.parallel(m_intakeRollers.stop(), m_channel.stop(), m_eeRollers.stop()),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION));
    }

    public Command intakeAlgae() {
        return Commands.sequence(
                m_eeWrist.moveTo(EndEffectorWristPosition.INTAKE_ALGAE_ANGLE),
                Commands.waitUntil(() -> m_eeRollers.isStalled()),
                m_eeRollers.stop(),
                m_eeRollers.holdAlgae());
    }

    public Command intake() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, intakeCoral()),
                        Map.entry(GPMode.Algae, intakeAlgae())),
                this::getGPMode);
    }

    public Command scoreBargeSequence() {
        // todo: figure out what to put here
        return null;
    }

    public Command moveL1() {
        // return Commands.runOnce(() -> {
        // System.out.println("hi");
        // });
        System.out.println("skibidi");
        System.out.println(m_elevator);
        System.out.println(m_eeWrist);
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.L1_CORAL_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.L1_SCORE_ANGLE))),
                        Map.entry(GPMode.Algae, Commands.parallel(
                                m_elevator.moveTo(ElevatorConstants.PROCESSOR_ALGAE_HEIGHT),
                                m_eeWrist.moveTo(EndEffectorWristPosition.SCORE_PROCESSOR_ANGLE)))),
                this::getGPMode);
    }

    public Command moveL2() {
        return Commands.runOnce(() -> {
            System.out.println("hi");
        });
        // return new SelectCommand<>(
        // Map.ofEntries(
        // Map.entry(GPMode.Coral, Commands.parallel(
        // m_elevator.moveTo(ElevatorConstants.L2_CORAL_HEIGHT),
        // m_eeWrist.moveTo(EndEffectorWristPosition.L2_PRE_ANGLE))),
        // Map.entry(GPMode.Algae, Commands.parallel(
        // m_elevator.moveTo(ElevatorConstants.L2_ALGAE_HEIGHT),
        // intakeAlgae()))),
        // this::getGPMode);
    }

    public Command moveL3() {
        return Commands.runOnce(() -> {
            System.out.println("hi");
        });
        // return new SelectCommand<>(
        // Map.ofEntries(
        // Map.entry(GPMode.Coral, Commands.parallel(
        // m_elevator.moveTo(ElevatorConstants.L3_CORAL_HEIGHT),
        // m_eeWrist.moveTo(EndEffectorWristPosition.L3_PRE_ANGLE))),
        // Map.entry(GPMode.Algae, Commands.parallel(
        // m_elevator.moveTo(ElevatorConstants.L3_ALGAE_HEIGHT),
        // intakeAlgae()))),
        // this::getGPMode);
    }

    public Command moveL4() {
        return Commands.runOnce(() -> {
            System.out.println("hi");
        });
        // return new SelectCommand<>(
        // Map.ofEntries(
        // Map.entry(GPMode.Coral, Commands.parallel(
        // m_elevator.moveTo(ElevatorConstants.L4_CORAL_HEIGHT),
        // m_eeWrist.moveTo(EndEffectorWristPosition.L4_PRE_ANGLE))),
        // Map.entry(GPMode.Algae, scoreBargeSequence())),
        // this::getGPMode);
    }

    public Command scoreCoral() {
        return Commands.runOnce(() -> {
            switch (m_eeWrist.getPosition()) {
                case L1_SCORE_ANGLE -> m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L1_CORAL_VOLTS);
                case L2_PRE_ANGLE, L3_PRE_ANGLE ->
                    m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L2_L3_CORAL_VOLTS);
                case L4_PRE_ANGLE -> m_eeRollers.run(EndEffectorConstants.Rollers.OUTTAKE_L4_CORAL_VOLTS);
                default -> m_eeRollers.run(0);
            }
            m_eeWrist.moveToNextPosition();
        });
    }

    public Command scoreAlgae() {
        return Commands.sequence(
                m_eeRollers.run(switch (m_eeWrist.getPosition()) {
                    case SCORE_PROCESSOR_ANGLE -> EndEffectorConstants.Rollers.OUTTAKE_PROCCESOR_VOLTS;
                    case SCORE_BARGE_ANGLE -> EndEffectorConstants.Rollers.OUTTAKE_BARGE_VOLTS;
                    default -> 0;
                }),
                m_eeRollers.stopHoldingAlgae());
    }

    public Command score() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(GPMode.Coral, scoreCoral()),
                        Map.entry(GPMode.Algae, scoreAlgae())),
                this::getGPMode);
    }

    public Command stow() {
        return Commands.parallel(
                // m_elevator.moveTo(ElevatorConstants.STOWED_HEIGHT),
                // m_eeWrist.moveTo(EndEffectorWristPosition.STOW_ANGLE),
                m_intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION),
                m_intakeRollers.stop(),
                // m_eeRollers.stop(),
                m_channel.stop());
    }
}
