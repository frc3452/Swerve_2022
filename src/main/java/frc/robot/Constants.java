// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ROBOT_LENGTH = 30;
    public static final double ROBOT_WIDTH = 30;
    public static final double DEFAULT_DEADBAND = 0.15;
    public static final double WHEEL_DIAMETER = 0.245;


    public final class C_SwerveModules {

        // Front Left Swerve Module
        public static final int FRONT_LEFT_AZIMUTH = 21;
        public static final int FRONT_LEFT_SPEED = 11;

        // Front Right Swerve Module
        public static final int FRONT_RIGHT_AZIMUTH = 22;
        public static final int FRONT_RIGHT_SPEED = 12;

        // Back Left Swerve Module
        public static final int BACK_LEFT_AZIMUTH = 23;
        public static final int BACK_LEFT_SPEED = 13;

        // Back Right Swerve Module
        public static final int BACK_RIGHT_AZIMUTH = 24;
        public static final int BACK_RIGHT_SPEED = 14;
    }

    public final class C_Climber {
        public static final int LEFT_CLIMBER = 31;
        public static final int RIGHT_CLIMBER = 32;
    }

    public final class C_Intake {
        public static final int INTAKE = 41;
    }

    public final class C_Index {
        public static final int LOWER_INDEX = 42;
        public static final int UPPER_INDEX = 43;
    }

    public final class C_Shooter {
        public static final int LEFT_SHOOTER = 44;
        public static final int RIGHT_SHOOTER = 45;
    }

    public final class C_Actuator {
        public static final int ACTUATOR = 46;
    }

    public final class C_Limelight {
        public static final double MOUNT_ANGLE = 24.03;
        public static final double DISTANCE_FROM_FLOOR = 86.36;
        public static final double GOAL_HEIGHT = 264.0;
    }

    public static final class C_FIELD_COORDINATES {
        public static final Translation2d ORIGIN = 
        new Translation2d(0, 0).plus(
            new Translation2d(1, Rotation2d.fromDegrees(56))); //Location of the center of the field + a Translation2d with distance and angle

        private static final double ball_radius = 3.8862;
        public static final Translation2d CENTER = new Translation2d(8.2296,4.1148); //in meters

        public static final Translation2d FIRST = new Translation2d(ball_radius, Rotation2d.fromDegrees(33.8));
        public static final Translation2d SECOND = new Translation2d(ball_radius, Rotation2d.fromDegrees(101.41));
        public static final Translation2d THIRD = new Translation2d(ball_radius, Rotation2d.fromDegrees(146.10));
        public static final Translation2d FOURTH = new Translation2d(305.66, Rotation2d.fromDegrees(66));

        
    }
}
