// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.swerve.WheelDrive;

import java.util.List;
import java.util.stream.Collectors;

public class SwerveDrive implements Subsystem {
    private final WheelDrive backRight;
    private final WheelDrive backLeft;
    private final WheelDrive frontRight;
    private final WheelDrive frontLeft;

    private final List<WheelDrive> modules;

    private final AHRS gyro;
    private Rotation2d gyroAngle;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Object pose;

    public SwerveDrive() {
        backRight = new WheelDrive(0, Constants.C_SwerveModules.BACK_RIGHT_AZIMUTH,
                Constants.C_SwerveModules.BACK_RIGHT_SPEED);
        backLeft = new WheelDrive(1, Constants.C_SwerveModules.BACK_LEFT_AZIMUTH,
                Constants.C_SwerveModules.BACK_LEFT_SPEED);
        frontRight = new WheelDrive(2, Constants.C_SwerveModules.FRONT_RIGHT_AZIMUTH,
                Constants.C_SwerveModules.FRONT_RIGHT_SPEED);
        frontLeft = new WheelDrive(3, Constants.C_SwerveModules.FRONT_LEFT_AZIMUTH,
                Constants.C_SwerveModules.FRONT_LEFT_SPEED);
        gyro = new AHRS();

        modules = List.of(backRight, backLeft, frontRight, frontLeft);

        var w = Constants.ROBOT_WIDTH / 2.0;
        var l = Constants.ROBOT_LENGTH / 2.0;
        Translation2d[] moduleLocations = { new Translation2d(-l, -w),
                new Translation2d(-l, w),
                new Translation2d(l, -w),
                new Translation2d(l, w) };

        kinematics = new SwerveDriveKinematics(
                moduleLocations);

        gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle);
    }

    @Override
    public void periodic() {
        modules.forEach(WheelDrive::update);
        gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());

        pose = odometry.update(gyroAngle,
                modules.stream().map(WheelDrive::getCurrentState).toArray(SwerveModuleState[]::new));
    }

    public void stop() {
        drive(new ChassisSpeeds(), false);
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldCentric) {
        ChassisSpeeds relativeSpeeds;
        if (fieldCentric) {
            relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    chassisSpeeds.omegaRadiansPerSecond,
                    gyroAngle);
        } else {
            relativeSpeeds = chassisSpeeds;
        }

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(relativeSpeeds);

            
        SmartDashboard.putString("States", 
        List.of(swerveModuleStates)
            .stream()
            .map(e -> e.angle.toString())
            .collect(Collectors.toList()).toString());
        for (int i = 0; i < swerveModuleStates.length; i++) {
            SwerveModuleState state = swerveModuleStates[i];
            modules.get(i).drive(state);
        }
    }

    public void zeroAzimuth() {
        modules.forEach(WheelDrive::zeroAzimuth);
    }
}