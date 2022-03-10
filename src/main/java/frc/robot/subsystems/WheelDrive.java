// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class WheelDrive {
    private TalonSRX azimuth;
    private CANSparkMax speedMotor;
    double Position = 0;

    public WheelDrive(int azimuth, int speedMotor) {
        this.azimuth = new TalonSRX(azimuth);
        this.azimuth.configFactoryDefault();
        this.azimuth.config_kP(0, 1);
        this.azimuth.config_kD(0, 0);
        this.azimuth.setInverted(false);
        this.azimuth.configPeakCurrentLimit(30);
        this.azimuth.setNeutralMode(NeutralMode.Coast);

        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.speedMotor.setInverted(false);
        this.speedMotor.setIdleMode(IdleMode.kBrake);
        this.speedMotor.setSmartCurrentLimit(1);
        this.speedMotor.setSmartCurrentLimit(40);
    }

    public void drive(double speed, double newPosition, String name) {

        if (speed == 0) {
            // Position = optimization(Position, 0);
            // this.speedMotor.setInverted(false);
        } else {
            // if (Math.abs((Position % 360) - (newPosition % 360)) > 60){
            Position = Position + optimization(Position, newPosition);
            // }
        }

        speedMotor.set(speed);
        azimuth.set(ControlMode.Position, Position * (4096 / 360));
    }

    private double optimization(double a, double b) {
        double dir = ((b % 360 + 360) % 360) - ((a % 360 + 360) % 360);

        if (Math.abs(dir) > 180) {
            dir = -(Math.signum(dir) * 360) + dir;
        }

        if (Math.abs(dir) > 90) {
            dir = -(180 - dir);
        }

        if (Math.abs(b) > 90) {
            this.speedMotor.setInverted(true);
        } else {
            this.speedMotor.setInverted(false);
        }
        return (dir % 360);
    }

    public void zeroAzimuth(String name) {
        this.azimuth.setSelectedSensorPosition(0);
        System.out.println(name);
    }

    public void autonomous(double rotations){
        this.speedMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature,1);
    }
}
