// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.swerve.SwerveDrive;

public class LimelightCommand extends CommandBase {
  private final Limelight limelight;

  public LimelightCommand(Limelight limelight, SwerveDrive swerve) {
    this.limelight = limelight;
    
    addRequirements(limelight, swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelight.getMode() == 1) {
      limelight.setMode(3);
    }else if (limelight.getMode() == 0 || limelight.getMode() == 3) {}

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = limelight.calculateDistance();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
