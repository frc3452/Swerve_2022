// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.swerve.SwerveDrive;

public class MoveToPosition extends CommandBase {
  private PIDController pid;
  /** Creates a new MoveToPosition. */
  private SwerveDrive swerve;
  private Pose2d current;
  private Pose2d target;

  double outputX;
  double outputY;
  double outputOmega;
  boolean positionDone;
  boolean speedDone;
  private Pose2d error = new Pose2d(new Translation2d(0,0), new Rotation2d(0));

  public MoveToPosition(SwerveDrive swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;
    pid = new PIDController(100.0, 0.0, 50.0);
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
    outputX = pid.calculate(current.getX(), target.getX());
    outputY = pid.calculate(current.getY(), target.getY());
    outputOmega = pid.calculate(current.getRotation().getDegrees(), target.getRotation().getDegrees());

    // translationError = target.getTranslation().minus(current.getTranslation());
    // rotationError = target.getRotation().minus(current.getRotation());
    error = new Pose2d(
        target.getTranslation().minus(current.getTranslation()),
        target.getRotation().minus(current.getRotation()));

    System.out.println(error.getTranslation());
    System.out.println(error.getRotation());
   

    swerve.drive(new ChassisSpeeds(outputX, outputY, outputOmega), true);
  }

  
  //public void periodic() {
    
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var speeds = swerve.getCurrentSpeed();
    positionDone =
    error.getTranslation().getNorm() < 0.1
        && error.getRotation().getDegrees() < 3;
    speedDone = speeds.vxMetersPerSecond < 0.1
        && speeds.vyMetersPerSecond < 0.1
        && speeds.omegaRadiansPerSecond < 3;
    return positionDone && speedDone;
  }
}
