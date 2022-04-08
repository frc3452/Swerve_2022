// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem{
  private CANSparkMax climberLeft;
  private CANSparkMax climberRight;
  private SparkMaxPIDController pidControllerClimberLeft;
  private SparkMaxPIDController pidControllerClimberRight;
  private RelativeEncoder EncoderClimberLeft;
  private RelativeEncoder EncoderClimberRight;

  public Climber(int leftClimber, int rightClimber) {
    this.climberLeft = new CANSparkMax(leftClimber, MotorType.kBrushless);
    this.climberLeft.setSmartCurrentLimit(40);
    this.climberLeft.setInverted(false);
    this.climberLeft.setIdleMode(IdleMode.kBrake);

    this.pidControllerClimberLeft = this.climberLeft.getPIDController();
    this.pidControllerClimberLeft.setP(1e-4);
    this.pidControllerClimberLeft.setFF(0.000195);

    this.EncoderClimberLeft = this.climberLeft.getEncoder();

    this.climberRight = new CANSparkMax(rightClimber, MotorType.kBrushless);
    this.climberRight.setSmartCurrentLimit(40);
    this.climberRight.setInverted(false);
    this.climberRight .setIdleMode(IdleMode.kBrake);

    this.pidControllerClimberRight = this.climberRight.getPIDController();
    this.pidControllerClimberRight.setP(1e-4);
    this.pidControllerClimberRight.setFF(0.000195);

    this.EncoderClimberRight = this.climberRight.getEncoder();
  }

  public void climber(double leftSpeed, double rightSpeed) {
    this.climberLeft.set(leftSpeed);
    this.climberRight.set(rightSpeed);

  }
}
