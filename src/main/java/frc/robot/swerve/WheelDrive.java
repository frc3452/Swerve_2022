// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;
import frc.robot.util.EncoderUnitConverter;
import frc.robot.util.SwerveUtil;

/**
 * Add your docs here.
 */
public class WheelDrive {
    private final int modulePosition;

    private final TalonSRX azimuth;
    private final CANSparkMax speedMotor;
    private SwerveModuleState currentState;

    private final SparkMaxPIDController drivePID;
    private final RelativeEncoder driveEncoder;

    private final EncoderUnitConverter azimuthConverter, driveConverter;

    // double Position;
    private double azimuthOffset;

    public WheelDrive(int module, int azimuth, int speedMotor) {
        modulePosition = module;

        azimuthConverter = new EncoderUnitConverter(1.0,
                4096.0,
                0.100,
                1023,
                Double.NaN);

        driveConverter = new EncoderUnitConverter(
                3.0,
                1.0,
                60.0,
                1.0,
                Units.inchesToMeters(2.0 + (15.0 / 16.0)) / 2.0);

        this.azimuth = new TalonSRX(azimuth);
        this.azimuth.configFactoryDefault();
        this.azimuth.config_kP(0, 3.5);
        this.azimuth.config_kD(0, 80);
        this.azimuth.setInverted(false);

        this.azimuth.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 100);

        this.azimuth.configContinuousCurrentLimit(30);
        this.azimuth.configPeakCurrentLimit(15);
        this.azimuth.configPeakCurrentDuration(50);

        this.azimuth.setNeutralMode(NeutralMode.Coast);

        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.speedMotor.restoreFactoryDefaults();
        this.speedMotor.setInverted(false);
        this.speedMotor.setSmartCurrentLimit(40);
        this.speedMotor.setIdleMode(IdleMode.kBrake);

        this.drivePID = this.speedMotor.getPIDController();
        this.drivePID.setP(0.1);
        this.drivePID.setD(1);

        this.driveEncoder = this.speedMotor.getEncoder();

        azimuthOffset = Preferences.getDouble(String.valueOf(modulePosition), -3452);
    }

    void update() {
        double azimuthTicks = this.azimuth.getSelectedSensorPosition() - azimuthOffset;
        double driveTicks = driveEncoder.getVelocity();
        currentState = new SwerveModuleState(driveConverter.velocity_tick_to_unit(driveTicks),
                new Rotation2d(azimuthConverter.position_tick_to_unit(azimuthTicks)));
        // if (modulePosition == 2) System.out.println(currentState);
    }

    SwerveModuleState getCurrentState() {
        return currentState;
    }

    public void drive(SwerveModuleState desired) {
        if (modulePosition == 2)
            System.out.println(desired);
        var optimized = SwerveUtil.optimization(currentState, desired);
        azimuth.set(
                ControlMode.Position,
                azimuthConverter.position_unit_to_tick(optimized.angle.getDegrees()));

        var percent = optimized.speedMetersPerSecond / Units.inchesToMeters(297);
        drivePID.setReference(percent, CANSparkMax.ControlType.kDutyCycle);

        // System.out.println();
        // drivePID.setReference(
        // driveConverter.velocity_unit_to_tick(optimized.speedMetersPerSecond),
        // CANSparkMax.ControlType.kVelocity);
    }

    public void zeroAzimuth() {
        azimuthOffset = this.azimuth.getSelectedSensorPosition() % 4096;
        if (azimuthOffset < 0)
            azimuthOffset += 4096;
        Preferences.setDouble(String.valueOf(modulePosition), azimuthOffset);
    }
}
