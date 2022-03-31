package frc.robot.util;

import frc.robot.Constants;

public class Util {

    public static double deadband(double value) {
        return deadband(value, Constants.DEFAULT_DEADBAND);
    }

    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
}
