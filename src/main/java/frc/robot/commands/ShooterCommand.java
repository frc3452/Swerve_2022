// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
  // private final XboxController joystick;
  private final Shooter shooter;

  public ShooterCommand(Shooter shooter) {

    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  }

 public void execute() {
  shooter.shoot();
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
