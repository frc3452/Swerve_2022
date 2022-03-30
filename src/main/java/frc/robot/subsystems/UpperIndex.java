// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperIndex extends SubsystemBase {
  private TalonSRX upperIndex;
  /** Creates a new Shooter. */
  public UpperIndex(int upperIndex) {
    this.upperIndex = new TalonSRX(upperIndex);
    this.upperIndex.setInverted(true);
  }

  public void index(double indexSpeed)
   {
    this.upperIndex.set(ControlMode.PercentOutput, indexSpeed);
  }

  public void stop() {
    this.upperIndex.set(ControlMode.PercentOutput, 0);
  }
}
