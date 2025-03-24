package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefPositions {
    public static final Distance REEF_CENTER_TO_BRANCH = Inches.of(6.633);
    public static final Distance BEHIND_REEF_CENTER = Inches.of(18);

    public static Pose2d getAlignPose(int tagId, boolean invertOffset) {
        Pose2d tagPose = VisionConstants.FIELD_TAG_LAYOUT.getTagPose(tagId).get().toPose2d();
        Translation2d alignOffsetRel = new Translation2d(
                -BEHIND_REEF_CENTER.in(Meters),
                ((invertOffset ? -1 : 1) * REEF_CENTER_TO_BRANCH.in(Meters)));
        Translation2d alignOffset = alignOffsetRel.rotateBy(tagPose.getRotation().minus(new Rotation2d(Math.PI)));
        Translation2d alignTrans = tagPose.getTranslation().plus(alignOffset);
        Pose2d alignPose = new Pose2d(alignTrans, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
        return alignPose;
    }

    static Map<AutoAlignPosition, Pose2d> reefPositionsBlue = Map.ofEntries(
            Map.entry(AutoAlignPosition.A, getAlignPose(18, false)),
            Map.entry(AutoAlignPosition.B, getAlignPose(18, true)),
            Map.entry(AutoAlignPosition.C, getAlignPose(17, false)),
            Map.entry(AutoAlignPosition.D, getAlignPose(17, true)),
            Map.entry(AutoAlignPosition.E, getAlignPose(22, false)),
            Map.entry(AutoAlignPosition.F, getAlignPose(22, true)),
            Map.entry(AutoAlignPosition.G, getAlignPose(21, false)),
            Map.entry(AutoAlignPosition.H, getAlignPose(21, true)),
            Map.entry(AutoAlignPosition.I, getAlignPose(20, false)),
            Map.entry(AutoAlignPosition.J, getAlignPose(20, true)),
            Map.entry(AutoAlignPosition.K, getAlignPose(19, false)),
            Map.entry(AutoAlignPosition.L, getAlignPose(19, true)));

    static Map<AutoAlignPosition, Pose2d> reefPositionsRed = Map.ofEntries(
            Map.entry(AutoAlignPosition.A, getAlignPose(7, false)),
            Map.entry(AutoAlignPosition.B, getAlignPose(7, true)),
            Map.entry(AutoAlignPosition.C, getAlignPose(8, false)),
            Map.entry(AutoAlignPosition.D, getAlignPose(8, true)),
            Map.entry(AutoAlignPosition.E, getAlignPose(9, false)),
            Map.entry(AutoAlignPosition.F, getAlignPose(9, true)),
            Map.entry(AutoAlignPosition.G, getAlignPose(10, false)),
            Map.entry(AutoAlignPosition.H, getAlignPose(10, true)),
            Map.entry(AutoAlignPosition.I, getAlignPose(11, false)),
            Map.entry(AutoAlignPosition.J, getAlignPose(11, true)),
            Map.entry(AutoAlignPosition.K, getAlignPose(6, false)),
            Map.entry(AutoAlignPosition.L, getAlignPose(6, true)));

    public static Pose2d getReefPosition(Alliance alliance, AutoAlignPosition position) {
        return alliance == Alliance.Blue ? reefPositionsBlue.get(position) : reefPositionsRed.get(position);
    }
}
