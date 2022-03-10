// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class SwerveDrive implements Subsystem {
    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;
    private AHRS gyro;

    public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.gyro = new AHRS();
    }

    public final static double L = Constants.ROBOT_LENGTH;
    public final static double W = Constants.ROBOT_WIDTH;
    public final static boolean isFieldOriented = true;
    
    double angle = 0;
    double offset = /*Preferences.getDouble("offset", 0)*/0;

    @Override
    public void periodic() {
        angle = (this.gyro.getAngle() - offset)%360;
    }


    public void drive(double pitch, double roll, double yaw) {
        double x1 = pitch;
        double y1 = roll;
        double x2 = yaw;

       if (isFieldOriented) {
        Translation2d translate = new Translation2d(x1,y1);
        Translation2d newCoords = translate.rotateBy(new Rotation2d(angle));
        x1 = newCoords.getX();
        y1 = newCoords.getY();
        }

        double r = Math.sqrt((L * L) + (W * W));

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        double backLeftAngle = Math.atan2(a, d) / Math.PI * 180;
        double backRightAngle = Math.atan2(a, c) / Math.PI * 180;
        double frontLeftAngle = Math.atan2(b, d) / Math.PI * 180;
        double frontRightAngle = Math.atan2(b, c) / Math.PI * 180;

        frontLeft.drive(frontLeftSpeed, frontLeftAngle, "1");
        frontRight.drive(frontRightSpeed, frontRightAngle, "2");
        backLeft.drive(backLeftSpeed, backLeftAngle, "3");
        backRight.drive(backRightSpeed, backRightAngle, "4");
    }

    /*public void zeroGyro() {
        gyro.setAngleAdjustment(0);
        double adj = gyro.getAngle() % 360;
        gyro.setAngleAdjustment(-adj);
    }*/

    public void zeroAzimuth() {
        frontLeft.zeroAzimuth("1");
        frontRight.zeroAzimuth("2");
        backLeft.zeroAzimuth("3");
        backRight.zeroAzimuth("4");
    }

    public void autonomous(double distance) {
        double r = 0;
        double rotations = (Math.PI*r*r)/distance/360;

        frontLeft.autonomous(rotations);
        frontRight.autonomous(rotations);
        backLeft.autonomous(rotations);
        backRight.autonomous(rotations);
    }
}