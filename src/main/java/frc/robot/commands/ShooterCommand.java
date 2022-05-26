// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {

  private final Shooter shooter;
  // private final boolean stop;
  private boolean direction;
  private boolean stop;
  
  public ShooterCommand(Shooter shooter, boolean direction) {
    this.shooter = shooter;
    this.direction = direction;
    addRequirements(shooter);
    this.stop = false;
  }

  public ShooterCommand(Shooter shooter, boolean direction, boolean stop) {
    this.shooter = shooter;
    this.direction = direction;
    addRequirements(shooter);
    this.stop = stop;
  }


  @Override
  public void initialize() {
  }

  public void execute() {
    if (!stop){
    shooter.shoot(!direction);
    } else {
      shooter.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
      shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
