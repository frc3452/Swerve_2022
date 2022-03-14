// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterLeft;
  private CANSparkMax shooterRight;
  /** Creates a new Shooter. */
  public Shooter(int shooterLeft, int shooterRight) {
    this.shooterLeft = new CANSparkMax(shooterLeft, MotorType.kBrushless);
    this.shooterLeft.setInverted(true);
    this.shooterLeft.setIdleMode(IdleMode.kCoast);
    // this.shooterLeft.set(30);

    this.shooterRight = new CANSparkMax(shooterRight, MotorType.kBrushless);
    this.shooterRight.setIdleMode(IdleMode.kCoast);
    // this.shooterRight.set(16);

  }

  public void shooter(double leftDecrease, double rightDecrease) {
    this.shooterLeft.set(Preferences.getDouble("frontSpeed", .3));
    this.shooterRight.set(Preferences.getDouble("backSpeed", .6));
  }

  public void stop() {
    this.shooterLeft.set(0);
    this.shooterRight.set(0);
  }
}
