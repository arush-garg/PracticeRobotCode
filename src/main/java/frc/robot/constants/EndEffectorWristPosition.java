package frc.robot.constants;

public enum EndEffectorWristPosition {
    L1_PRE_ANGLE(0.0),
    L2_PRE_ANGLE(0.0),
    L3_PRE_ANGLE(0.0),
    L4_PRE_ANGLE(0.0),
    L1_SCORE_ANGLE(0.0),
    L2_SCORE_ANGLE(0.0),
    L3_SCORE_ANGLE(0.0),
    L4_SCORE_ANGLE(0.0),
    DEALGAE_HIGH_ANGLE(0.0),
    DEALGAE_LOW_ANGLE(0.0),
    STOW_WRIST_ANGLE(0.0),
    SCORE_BARGE_ANGLE(0.0),
    SCORE_PROCESSOR_ANGLE(0.0),
    INTAKE_ANGLE(0.0);

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
