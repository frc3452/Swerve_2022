// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperIndex;

public class UpperIndexCommand extends CommandBase {
  private final UpperIndex index;

  public UpperIndexCommand(UpperIndex index) {
    this.index = index;
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    index.index();
  }

  @Override
  public void end(boolean interrupted) {
    index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
