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
        if (speed != 0) {
            // Position = Position + optimization(Position, newPosition);
            // System.out.println(optimization(Position, newPosition));
            if (name == "1") {
                // System.out.println(speed);
            }
            // this.azimuth.set(ControlMode.Position, Position *(4096/360));
            this.azimuth.set(ControlMode.Position, newPosition *(4096.0/360.0));
        }
        speedMotor.set(speed);
    }

    private double optimization(double a, double b) {
        double dir = Math.IEEEremainder(b, 360) - Math.IEEEremainder(a, 360);
System.out.println(!this.speedMotor.getInverted());
        if (Math.abs(dir) > 180) {
            dir = -(Math.signum(dir) * 360) + dir;
        }

        if (Math.abs(dir) > 90) {
            dir = Math.signum(dir) * (180 - Math.abs(dir));
                this.speedMotor.setInverted(!this.speedMotor.getInverted());
        }

        return dir;
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
        // this.pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        // System.out.println("Encoder position end ");
        // System.out.println(this.encoder.getPosition());
        
    }
}
