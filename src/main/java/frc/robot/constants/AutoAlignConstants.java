package frc.robot.constants;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AutoAlignConstants {
    public static final PathConstraints PATH_PLANNER_CONSTRAINTS = new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final double FORWARDS_SPEED = 0.9; // speed until stall
    public static final double CURRENT_THRESHOLD = 70; // current of stall threshold

    // private static final Rotation2d AB = new Rotation2d(Math.toRadians(0));
    // private static final Rotation2d CD = new Rotation2d(Math.toRadians(60));
    // private static final Rotation2d EF = new Rotation2d(Math.toRadians(120));
    // private static final Rotation2d GH = new Rotation2d(Math.toRadians(180));
    // private static final Rotation2d IJ = new Rotation2d(Math.toRadians(240));
    // private static final Rotation2d KL = new Rotation2d(Math.toRadians(300));
    private static final Rotation2d AB = new Rotation2d(Math.toRadians(0));
    private static final Rotation2d CD = new Rotation2d(Math.toRadians(60));
    private static final Rotation2d EF = new Rotation2d(Math.toRadians(120));
    private static final Rotation2d GH = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d IJ = new Rotation2d(Math.toRadians(240));
    private static final Rotation2d KL = new Rotation2d(Math.toRadians(300));

    public static final Pose2d REEF_A = new Pose2d(2.85, 4.21, AB);
    public static final Pose2d REEF_B = new Pose2d(2.85, 3.77, AB);
    public static final Pose2d REEF_C = new Pose2d(3.59, 2.69, CD);
    public static final Pose2d REEF_D = new Pose2d(3.89, 2.54, CD);
    public static final Pose2d REEF_E = new Pose2d(5.13, 2.53, EF);
    public static final Pose2d REEF_F = new Pose2d(5.47, 2.74, EF);
    public static final Pose2d REEF_G = new Pose2d(6.09, 3.81, GH);
    public static final Pose2d REEF_H = new Pose2d(6.09, 4.21, GH);
    public static final Pose2d REEF_I = new Pose2d(5.39, 5.36, IJ);
    public static final Pose2d REEF_J = new Pose2d(5.06, 5.57, IJ);
    public static final Pose2d REEF_K = new Pose2d(3.82, 5.51, KL);
    public static final Pose2d REEF_L = new Pose2d(3.39, 5.48, KL);

    public static final Pose2d ALGAE_AB = new Pose2d(3.1876, 4.026, AB);
    public static final Pose2d ALGAE_CD = new Pose2d(3.839, 2.899, CD);
    public static final Pose2d ALGAE_EF = new Pose2d(5.1396, 2.899, EF);
    public static final Pose2d ALGAE_GH = new Pose2d(5.7909, 4.0258, GH);
    public static final Pose2d ALGAE_IJ = new Pose2d(5.1396, 5.15242, IJ);
    public static final Pose2d ALGAE_KL = new Pose2d(3.8389, 5.1524, KL);

    public static final ArrayList<Pose2d> ALL_REEF_POSES = new ArrayList<Pose2d>() {
        {
            add(REEF_A);
            add(REEF_B);
            add(REEF_C);
            add(REEF_D);
            add(REEF_E);
            add(REEF_F);
            add(REEF_G);
            add(REEF_H);
            add(REEF_I);
            add(REEF_J);
            add(REEF_K);
            add(REEF_L);
        }
    };
    public static final HashMap<AutoAlignPosition, Pose2d> REEF_POSITIONS = new HashMap<AutoAlignPosition, Pose2d>() {
        {
            put(AutoAlignPosition.A, REEF_A);
            put(AutoAlignPosition.B, REEF_B);
            put(AutoAlignPosition.C, REEF_C);
            put(AutoAlignPosition.D, REEF_D);
            put(AutoAlignPosition.E, REEF_E);
            put(AutoAlignPosition.F, REEF_F);
            put(AutoAlignPosition.G, REEF_G);
            put(AutoAlignPosition.H, REEF_H);
            put(AutoAlignPosition.I, REEF_I);
            put(AutoAlignPosition.J, REEF_J);
            put(AutoAlignPosition.K, REEF_K);
            put(AutoAlignPosition.L, REEF_L);
        }
    };

    public static final HashMap<AutoAlignPosition, Pose2d> ALGAE_POSITIONS = new HashMap<AutoAlignPosition, Pose2d>() {
        {
            put(AutoAlignPosition.A, ALGAE_AB);
            put(AutoAlignPosition.B, ALGAE_AB);
            put(AutoAlignPosition.C, ALGAE_CD);
            put(AutoAlignPosition.D, ALGAE_CD);
            put(AutoAlignPosition.E, ALGAE_EF);
            put(AutoAlignPosition.F, ALGAE_EF);
            put(AutoAlignPosition.G, ALGAE_GH);
            put(AutoAlignPosition.H, ALGAE_GH);
            put(AutoAlignPosition.I, ALGAE_IJ);
            put(AutoAlignPosition.J, ALGAE_IJ);
            put(AutoAlignPosition.K, ALGAE_KL);
            put(AutoAlignPosition.L, ALGAE_KL);
        }
    };

}
