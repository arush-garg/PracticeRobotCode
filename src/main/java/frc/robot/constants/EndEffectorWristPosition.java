package frc.robot.constants;

public enum EndEffectorWristPosition {
    // coral
    L2_PRE_ANGLE(0.0),
    L3_PRE_ANGLE(0.0),
    L4_PRE_ANGLE(0.0),
    L1_SCORE_ANGLE(0.0),
    L2_SCORE_ANGLE(0.0),
    L3_SCORE_ANGLE(0.0),
    L4_SCORE_ANGLE(0.0),
    INTAKE_CORAL_ANGLE(0.0), // should be the same as STOW_ANGLE
    // algae
    SCORE_BARGE_ANGLE(0.0),
    SCORE_PROCESSOR_ANGLE(0.0),
    INTAKE_ALGAE_ANGLE(0.0),
    // stow
    STOW_ANGLE(0.0);

    private final double angle;

    EndEffectorWristPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public double getBackAngle() {
        return -angle;
    }
}
