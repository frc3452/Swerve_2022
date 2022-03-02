// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class WheelDrive {
    private TalonSRX azimuth;
    private CANSparkMax speedMotor;
    double Position = 0;
    int Inverted = 1;

    public WheelDrive(int azimuth, int speedMotor) {
        this.azimuth = new TalonSRX(azimuth);
        this.azimuth.configFactoryDefault();
        this.azimuth.config_kP(0, 3.5);
        this.azimuth.config_kD(0, 80);
        this.azimuth.setInverted(false);
        this.azimuth.setNeutralMode(NeutralMode.Coast);

        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.speedMotor.setInverted(false);
        this.speedMotor.setIdleMode(IdleMode.kBrake);
    }

    public void drive(double speed, double newPosition, String name) {

        if (speed != 0) {
            Position = Position - optimization(Position, newPosition);
        }
        System.out.println(Position % 360);

        // speedMotor.set(speed * Inverted);
        // azimuth.set(ControlMode.Position, Position * (4096 / 360));
    }

    private double optimization(double a, double b) {
        double dir = (b % 360) - (a % 360);

        if (Math.abs(dir) > 180) {
            dir = -(Math.signum(dir) * 360) + dir;
        }

        if (Math.abs(dir) > 90) {
            dir = -(180 - dir);
            Inverted *= -1; 
        }

        return dir;
    }

    public void zeroAzimuth(String name) {
        Preferences.remove(name);
        Preferences.setDouble(name, this.azimuth.getSensorCollection().getPulseWidthPosition());
        this.azimuth.setSelectedSensorPosition(0);
        System.out.println(name);
    }

    public void getPreference(String name) {
        System.out.println(this.azimuth.getSensorCollection().getPulseWidthPosition());
    }
}
