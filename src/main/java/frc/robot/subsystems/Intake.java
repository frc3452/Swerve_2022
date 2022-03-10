// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonSRX intake;
  private TalonSRX lowerIndex;
  
  public Intake(int intake, int lowerIndex) {
    this.intake = new TalonSRX(intake);
    this.lowerIndex = new TalonSRX(lowerIndex);
  }

  public void intake(double intakeSpeed, double indexSpeed) {
    this.intake.set(ControlMode.PercentOutput,intakeSpeed);
    this.lowerIndex.set(ControlMode.PercentOutput, indexSpeed);
  }
  public void stop() {
    this.intake.set(ControlMode.PercentOutput,0);
    this.lowerIndex.set(ControlMode.PercentOutput, 0);
  }
}
