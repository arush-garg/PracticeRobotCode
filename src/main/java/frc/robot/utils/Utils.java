package frc.robot.utils;

public class Utils {

    public static boolean NearZero(double x) {
        if (x < 0.01 && x > -0.01) {
            return true;
        }
        return false;
    }

}


