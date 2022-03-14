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

import frc.robot.commands.ZeroAzimuthCommand;

/** Add your docs here. */
public class WheelDrive {
    private TalonSRX azimuth;
    private CANSparkMax speedMotor;
    double Position = 0;
    boolean i = true;
    int c=0;

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
        this.speedMotor.setSmartCurrentLimit(40);
    }

    public void drive(double speed, double newPosition, String name) {
        if (speed != 0) {
            // Position = Position + optimization(Position, newPosition);
            // System.out.println(optimization(Position, newPosition));
            if (name == "1") {
                // System.out.println(speed);
            }
            this.azimuth.set(ControlMode.Position, newPosition*(4096/360));
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
            // if () {
                this.speedMotor.setInverted(!this.speedMotor.getInverted());
                // c += 1;

            // }
            // if (c%2 == 0) {
            //     this.speedMotor.setInverted(!this.speedMotor.getInverted());

            

        //     System.out.println(this.speedMotor.getInverted());
        } else {
            c= 1;
        }

        return dir;
    }

    public void zeroAzimuth(String name) {
        this.azimuth.setSelectedSensorPosition(0);
        System.out.println(name);
    }
}
