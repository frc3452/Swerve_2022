// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class wpilibSwerveDrive extends SubsystemBase {
  /** Creates a new wpilibSwerveDrive. */
  public wpilibSwerveDrive() {

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Constants.ROBOT_LENGTH / 2, Constants.ROBOT_WIDTH / 2),
      // Front right
      new Translation2d(Constants.ROBOT_LENGTH / 2, Constants.ROBOT_WIDTH / 2),
      // Front left
      new Translation2d(Constants.ROBOT_LENGTH / 2, Constants.ROBOT_WIDTH / 2),
      // Front right
      new Translation2d(Constants.ROBOT_LENGTH / 2, Constants.ROBOT_WIDTH / 2)
    );

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
