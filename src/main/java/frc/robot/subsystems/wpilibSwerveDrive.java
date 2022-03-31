// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class wpilibSwerveDrive extends SubsystemBase {
  SwerveDriveKinematics m_kinematics;
  ChassisSpeeds speeds;
  /** Creates a new wpilibSwerveDrive. */
  public wpilibSwerveDrive() {

      Translation2d m_frontLeftModule = new Translation2d(Constants.ROBOT_WIDTH / 2, Constants.ROBOT_LENGTH / 2);
      Translation2d m_frontRightModule = new Translation2d(Constants.ROBOT_WIDTH / 2, Constants.ROBOT_LENGTH / 2);
      Translation2d m_backLeftModule = new Translation2d(Constants.ROBOT_WIDTH / 2, Constants.ROBOT_LENGTH / 2);
      Translation2d m_backRightModule = new Translation2d(Constants.ROBOT_WIDTH / 2, Constants.ROBOT_LENGTH / 2);

      m_kinematics = new SwerveDriveKinematics(
        m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule
      );

      speeds = new ChassisSpeeds(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double roll, double pitch, double yaw) {
    speeds = new ChassisSpeeds(roll, pitch,yaw);

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[2];
    SwerveModuleState backLeft = moduleStates[3];
    SwerveModuleState backRight = moduleStates[4];

  }
}
