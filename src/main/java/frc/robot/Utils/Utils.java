package frc.robot.Utils;

public class Utils {
    public static double deadband(double x, double d) {
        return ((x > d)) || (x < -d) ? x : 0.0;
    }

    public static double squarePreserveSign(double x) {
        if (x < 0) {
            return -x*x;
        }
        return x*x;
    }

    public static double cubePreserveSign(double x) {
        return x*x*x;
    }

}
