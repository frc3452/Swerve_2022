// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class WheelDrive {
    private TalonSRX azimuth;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;

    double Position = 0;

    public WheelDrive(int azimuth, int speedMotor) {

        this.azimuth = new TalonSRX(azimuth);
        this.azimuth.configFactoryDefault();
        this.azimuth.config_kP(0, 3.5);
        this.azimuth.config_kD(0, 80);
        this.azimuth.setInverted(false);
        this.azimuth.configPeakCurrentLimit(30);
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
    }

    public void drive(double speed, double newPosition, String name) {
        var optimized = optimization(new SwerveModuleState(speed, Rotation2d.fromDegrees(newPosition)));
        if (speed != 0) {
            this.azimuth.set(ControlMode.Position, optimized.angle.getDegrees() * (4096.0 / 360.0));
            if (name == "1") {
                System.out.println(optimized);
            }
        }
        speedMotor.set(optimized.speedMetersPerSecond);
    }

    void printAngle() {
        System.out.println(wheelState());
    }

    public Rotation2d wheelState() {
        return Rotation2d.fromDegrees(this.azimuth.getSelectedSensorPosition() / (4096.0 / 360.0));
    }

    private SwerveModuleState optimization(SwerveModuleState desiredState) {
        var current = wheelState();
        desiredState.angle = Rotation2d
                .fromDegrees(desiredState.angle.getDegrees() + current.getDegrees() - current.getDegrees() % 360.0);
        return SwerveModuleState.optimize(desiredState, current);
    }

    public void zeroAzimuth(String name) {
        this.azimuth.setSelectedSensorPosition(0);
        System.out.println(name);
    }

    public double returnDistance() {
        double dist_1_rev = Constants.WHEEL_DIAMETER * Math.PI;
        return dist_1_rev;
    }

    public void auto(double distance) {
        // double circumference = Constants.WHEEL_DIAMETER * Math.PI;
        // double rotations = distance/circumference;
        // System.out.println("Rotations desired: ");
        // System.out.println(rotations);
        // this.encoder.setPosition(0);
        // System.out.println("Encoder position start ");
        // System.out.println(this.encoder.getPosition());
        // this.encoder.setPositionConversionFactor(4096);
        // this.pidController.setReference(rotations,
        // CANSparkMax.ControlType.kPosition);
        // System.out.println("Encoder position end ");
        // System.out.println(this.encoder.getPosition());

    }
}
