// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.swerve.SwerveDrive;

public class ZeroAzimuthCommand extends InstantCommand {
  private final SwerveDrive swerve;

  /** Creates a new ZeroAzimuthCommand. */
  public ZeroAzimuthCommand(SwerveDrive swerve) {
    this.swerve = swerve;
    // addRequirements(swerve);
  }

  @Override
  public void initialize() {
      System.out.println("Zeroing azmiuth");
      swerve.zeroAzimuth();
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }
}
