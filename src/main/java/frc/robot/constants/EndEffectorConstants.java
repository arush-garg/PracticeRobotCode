package frc.robot.constants;

public final class EndEffectorConstants {

    public static class Wrist {
        public final static int MOTOR_ID = 0;

        public final static double MAX_VOLTS = 0.0;
        public final static double MANUAL_RATIO = 0.0;
        public final static double GEAR_RATIO = 0.0;

        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;
        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;
        public final static double kG = 0.0;

        public final static double MOTION_ACCELERATION = 0.0;
        public final static double MOTION_CRUISE_VELOCITY = 0.0;
        public final static double MOTION_JERK = 0.0;

        public final static double OFFSET = 0.0;
    }

    public enum WristPosition {
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
        SCORE_PROCESSOR_ANGLE(0.0);

        private final double angle;

        WristPosition(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    public static class Rollers {
        public final static int MOTOR_ID = 0;

        public final static double MAX_VOLTS = 0.0;
        public final static double INTAKE_CORAL_VOLTS = 0.0;
        public final static double INTAKE_ALGAE_VOLTS = 0.0;
        public final static double OUTTAKE_L2_L3_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_L1_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_L4_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_BARGE_VOLTS = 0.0;
        public final static double OUTTAKE_PROCCESOR_VOLTS = 0.0;
        public final static double RETAIN_ALGAE = 0.0;

    }
}
