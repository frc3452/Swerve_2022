// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.util.EncoderUnitConverter;
import frc.robot.util.SwerveUtil;

/**
 * Add your docs here.
 */
public class WheelDrive {
    private final int modulePosition;

    private final TalonSRX azimuth;
    public final CANSparkMax speedMotor;
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
                5.25,
                1.0,
                60.0,
                1.0,
                Units.inchesToMeters(2.0 + (15.0 / 16.0)) / 2.0);

        this.azimuth = new TalonSRX(azimuth);
        this.azimuth.configFactoryDefault();
        this.azimuth.config_kP(0, 3.5);
        this.azimuth.config_kD(0, 80);
        this.azimuth.setInverted(true);

        this.azimuth.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 100);
        this.azimuth.setSensorPhase(false);

        this.azimuth.configContinuousCurrentLimit(30);
        this.azimuth.configPeakCurrentLimit(15);
        this.azimuth.configPeakCurrentDuration(50);

        this.azimuth.setNeutralMode(NeutralMode.Coast);

        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.speedMotor.restoreFactoryDefaults();
        this.speedMotor.setInverted(false);


        // if(this.modulePosition == 2) {
        //     this.speedMotor.setInverted(true);
        // }

        this.speedMotor.setSmartCurrentLimit(40);
        this.speedMotor.setIdleMode(IdleMode.kBrake);

        this.drivePID = this.speedMotor.getPIDController();
        this.drivePID.setP(0.1);
        this.drivePID.setD(1);

        this.driveEncoder = this.speedMotor.getEncoder();

        azimuthOffset = Preferences.getDouble(String.valueOf(modulePosition), -3452);

        Shuffleboard.getTab("swervy").addNumber("Module " + module, new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                if (currentState != null) return currentState.speedMetersPerSecond;
                return 3452.0;
            }
        });
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
        // azimuth.set(ControlMode.PercentOutput, 0.1);
        // drivePID.setReference(0.1, CANSparkMax.ControlType.kDutyCycle);

        var optimized = SwerveUtil.optimization(currentState, desired);

        if (desired.speedMetersPerSecond != 0) {
        // var optimized = desired;
        azimuth.set(
                ControlMode.Position,
                azimuthConverter.position_unit_to_tick(optimized.angle.getRadians()) + azimuthOffset);
    }
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
