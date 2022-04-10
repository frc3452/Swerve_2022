// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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


  private Pose2d error;
  public MoveToPosition(SwerveDrive swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;
    pid = new PIDController(250.0,25.0,0.0);
    addRequirements(swerve);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {         
  }
  
  public void periodic() {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // var speeds = swerve.getCurrentSpeed();
    
    return false;
  }
}
