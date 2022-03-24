// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        var tab = Shuffleboard.getTab("SmartDashboard");

        tab.addString("BL", () -> backLeft.wheelState().toString());
        tab.addString("BR", () -> backRight.wheelState().toString());
        tab.addString("FL", () -> frontLeft.wheelState().toString());
        tab.addString("FR", () -> frontRight.wheelState().toString());
    }

    public final static double L = Constants.ROBOT_LENGTH;
    public final static double W = Constants.ROBOT_WIDTH;
    public static boolean isFieldOriented = true;

    double angle = 0;
    double offset = 0.0;

    @Override
    public void periodic() {
        angle = (this.gyro.getAngle() - offset) % 360;
        // frontRight.printAngle();
    }

    public void zero() {
        offset = this.gyro.getAngle();
    }

    public void drive(double x1, double y1, double theta) {

        // System.out.println(String.format("X: %.2f, Y: %.2f, T: %.2f", x1, y1, theta));
        if (isFieldOriented) {
            Translation2d translate = new Translation2d(x1, y1);
            Translation2d newCoords = translate.rotateBy(new Rotation2d(Math.toRadians(angle)));
            x1 = newCoords.getX();
            y1 = newCoords.getY();
        }

        // System.out.println(String.format("X: %.2f, Y: %.2f", x1, y1));

        double r = Math.sqrt((L * L) + (W * W));

        double a = x1 - theta * (L / r);
        double b = x1 + theta * (L / r);
        double c = y1 - theta * (W / r);
        double d = y1 + theta * (W / r);

        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        double backLeftAngle = Math.atan2(a, d) / Math.PI * 180;
        double backRightAngle = Math.atan2(a, c) / Math.PI * 180;
        double frontLeftAngle = Math.atan2(b, d) / Math.PI * 180;
        double frontRightAngle = Math.atan2(b, c) / Math.PI * 180;

        frontLeft.drive(frontLeftSpeed, frontLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        backRight.drive(backRightSpeed, backRightAngle);
    }

    /*
     * public void zeroGyro() {
     * gyro.setAngleAdjustment(0);
     * double adj = gyro.getAngle() % 360;
     * gyro.setAngleAdjustment(-adj);
     * }
     */

    public void zeroAzimuth() {
        frontLeft.zeroAzimuth();
        frontRight.zeroAzimuth();
        backLeft.zeroAzimuth();
        backRight.zeroAzimuth();
    }

    public void auto(double Distance) {
        // frontLeft.auto(Distance);
        // frontRight.auto(Distance);
        // backLeft.auto(Distance);
        backRight.auto(Distance);
    }
}