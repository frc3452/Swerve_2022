// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.swerve.SwerveDrive;

public class RotateToDegree extends CommandBase {

  private SwerveDrive swerve;
  private Pose2d current;
  private Pose2d newPose;
  private Rotation2d target;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(SwerveDrive swerve, Rotation2d target) {
    this.swerve = swerve;
    this.target = target;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current = swerve.getpose();
    newPose = new Pose2d(current.getTranslation(), target);
    new MoveToPosition(swerve, newPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.newSwerve();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
