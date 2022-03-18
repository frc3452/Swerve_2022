// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class WheelDrive {
    private final TalonSRX azimuth;
    private final int azimuthId;
    private final CANSparkMax speedMotor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;

    // double Position;

    private double azimuthOffset;

    public WheelDrive(int azimuth, int speedMotor) {
        azimuthId = azimuth;

        this.azimuth = new TalonSRX(azimuth);
        this.azimuth.configFactoryDefault();
        this.azimuth.config_kP(0, 3.5);
        this.azimuth.config_kD(0, 80);
        this.azimuth.setInverted(false);

        // this.azimuth.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.azimuth.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute,0,100);

        
        this.azimuth.configContinuousCurrentLimit(30);
        this.azimuth.configPeakCurrentLimit(15);
        this.azimuth.configPeakCurrentDuration(50);

        this.azimuth.setNeutralMode(NeutralMode.Coast);

        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.speedMotor.restoreFactoryDefaults();
        this.speedMotor.setInverted(false);
        this.speedMotor.setSmartCurrentLimit(40);
        this.speedMotor.setIdleMode(IdleMode.kBrake);

        this.pidController = this.speedMotor.getPIDController();
        this.pidController.setP(0.1);
        this.pidController.setD(1);

        this.encoder = this.speedMotor.getEncoder();

        azimuthOffset = Preferences.getDouble(String.valueOf(azimuthId), -3452);
    }

    public void drive(double speed, double newPosition) {
        if (azimuthId == 21) {
            System.out.println(wheelState() + String.format("Offset: %.2f", azimuthOffset));
        }
        var optimized = optimization(new SwerveModuleState(speed, Rotation2d.fromDegrees(newPosition)));
        if (speed != 0) {
            this.azimuth.set(ControlMode.Position, optimized.angle.getDegrees() * (4096.0 / 360.0) + azimuthOffset);
        }
        speedMotor.set(optimized.speedMetersPerSecond);
    }

    public double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
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

    public Rotation2d wheelState() {
        double tick = this.azimuth.getSelectedSensorPosition() - azimuthOffset;
        return Rotation2d.fromDegrees(tick / (4096.0 / 360.0));
    }

    private SwerveModuleState optimization(SwerveModuleState desiredState) {
        var current = wheelState();
        desiredState.angle = Rotation2d.fromDegrees(
                placeInAppropriate0To360Scope(current.getDegrees(),
                        desiredState.angle.getDegrees()));
        return SwerveModuleState.optimize(desiredState, current);
    }

    public void zeroAzimuth() {
        azimuthOffset = this.azimuth.getSelectedSensorPosition() % 4096;
        if (azimuthOffset < 0) azimuthOffset += 4096;
        Preferences.setDouble(String.valueOf(azimuthId), azimuthOffset);
    }

    public double returnDistance() {
        double dist_1_rev = Constants.WHEEL_DIAMETER * Math.PI;
        return dist_1_rev;
    }

    public void auto(double distance) {
        double circumference = Constants.WHEEL_DIAMETER * Math.PI;
        double rotations = distance/circumference;
        System.out.println("Rotations desired: ");
        System.out.println(rotations);
        this.encoder.setPosition(0);
        System.out.println("Encoder position start ");
        System.out.println(this.encoder.getPosition());
        // this.encoder.setPositionConversionFactor(42);
        this.pidController.setReference(rotations,
        CANSparkMax.ControlType.kPosition);
        System.out.println("Encoder position end ");
        System.out.println(this.encoder.getPosition());
    }
}
