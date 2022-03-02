// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem{
  private CANSparkMax climberLeft;
  private CANSparkMax climberRight;

  public Climber(int leftClimber, int rightClimber) {
    this.climberLeft = new CANSparkMax(leftClimber, MotorType.kBrushless);
    this.climberRight = new CANSparkMax(rightClimber, MotorType.kBrushless);
  }

  public void climber(double leftSpeed, double rightSpeed) {
    this.climberLeft.set(leftSpeed);
    this.climberRight.set(rightSpeed);

  }
}
