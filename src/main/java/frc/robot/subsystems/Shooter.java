// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterLeft;
  private CANSparkMax shooterRight;

  private TalonSRX upperIndex;
  /** Creates a new Shooter. */
  public Shooter(int shooterLeft, int shooterRight, int upperIndex) {
    this.shooterLeft = new CANSparkMax(shooterLeft, MotorType.kBrushless);
    this.shooterLeft.setInverted(true);
    this.shooterRight = new CANSparkMax(shooterRight, MotorType.kBrushless);

    this.upperIndex = new TalonSRX(upperIndex);
  }

  public void shooter(double leftDecrease, double rightDecrease) {
    this.shooterLeft.set((1-leftDecrease));
    this.shooterRight.set((1-rightDecrease));

    this.upperIndex.set(ControlMode.PercentOutput, 1);
  }

  public void stop() {
    this.shooterLeft.set(0);
    this.shooterRight.set(0);

    this.upperIndex.set(ControlMode.PercentOutput, 0);
  }

  
}
