// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.utils.Converters;

import frc.robot.util.Util;

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
    public static final double MAX_LINEAR_VELOCITY = 3;
    public static final double MAX_ANGULAR_VELOCITY = Math.PI;
    public static final double DEFAULT_DEADBAND = 0.3;
    public static final double WHEEL_DIAMETER = 0.245;


    public static final class C_SwerveModules {

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

    public static final class C_Climber {
        public static final int LEFT_CLIMBER = 31;
        public static final int RIGHT_CLIMBER = 32;
    }

    public static final class C_Intake {
        public static final int INTAKE = 41;
    }

    public static final class C_Index {
        public static final int LOWER_INDEX = 42;
        public static final int UPPER_INDEX = 43;
    }

    public static final class C_Shooter {
        public static final int LEFT_SHOOTER = 44;
        public static final int RIGHT_SHOOTER = 45;
    }

    public static final class C_Actuator {
        public static final int ACTUATOR = 46;
    }

    public static final class C_Limelight {
        public static final double MOUNT_ANGLE = 0.0;
        public static final double GOAL_HEIGHT = 0.0;
        public static final double DISTANCE_FROM_FLOOR =0.0;
    }

    public static final class C_FIELD_POSITIONS {
        
        public static final double FIRST_BALL_X = (7.64);
            // 300.05 * 0.0254) ;
        public static final double SECOND_BALL_X = (107.68 * 0.0254);
        public static final double THIRD_BALL_X = (201.75 * 0.0254);
        public static final double FOURTH_BALL_X = (201.3 * 0.0254);

        public static final double FIRST_BALL_Y = (0.0);
            // 7.05 * 0.0254) ;
        public static final double SECOND_BALL_Y = (16.5 * 0.0254) ;
        public static final double THIRD_BALL_Y = (33.05 * 0.0254) ;
        public static final double FOURTH_BALL_Y = (271.3 * 0.0254) ;
        

    }
}
