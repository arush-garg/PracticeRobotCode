package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveToPoseConstants {
    public static final Distance DRIVE_TOLERANCE = Meters.of(0.01);
    public static final Angle THETA_TOLERANCE = Degrees.of(1.0);

    public static final LinearVelocity DRIVE_MAX_VELOCITY = MetersPerSecond.of(1.0);
    public static final AngularVelocity THETA_MAX_VELOCITY = DegreesPerSecond.of(360);

    public static final LinearAcceleration DRIVE_MAX_ACCELERATION = MetersPerSecondPerSecond.of(2.0);
    public static final AngularAcceleration THETA_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(8);

    public static final Distance FF_MIN_RADIUS = Meters.of(0.02);
    public static final Distance FF_MAX_RADIUS = Meters.of(0.15);

    public static final PIDConstants DRIVE_PID = new PIDConstants(9.5, 0.0, 0);
    public static final PIDConstants THETA_PID = new PIDConstants(8.0, 0.0, 0);

    public static final LinearVelocity MAX_TRANSLATIONAL_VELOCITY = MetersPerSecond.of(4.5);
    public static final AngularVelocity MAX_ROTATIONAL_VELOCITY = RadiansPerSecond.of(12);
}
