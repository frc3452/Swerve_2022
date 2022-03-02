// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class ZeroAzimuthCommand extends CommandBase {
  private final SwerveDrive swerve;

  /** Creates a new ZeroAzimuthCommand. */
  public ZeroAzimuthCommand(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    //if (RobotState.isTest()) {
      System.out.println("Zeroing azmiuth");
      swerve.zeroAzimuth();
    // } else {
    //   System.out.println("Can't zero azimuth, not in test");
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
