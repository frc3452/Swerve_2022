package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtil {


    public static SwerveModuleState optimization(SwerveModuleState currentState, SwerveModuleState desiredState) {
        SwerveModuleState angleOptimized = SwerveModuleState.optimize(desiredState, currentState.angle);
        angleOptimized.angle = Rotation2d.fromDegrees(
                SwerveUtil.placeInAppropriate0To360Scope(currentState.angle.getDegrees(),
                        angleOptimized.angle.getDegrees()));
        return angleOptimized;
    }

    // public static Rotation2d autonOptimisation(Rotation2d current, Rotation2d target) {

    // }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}
