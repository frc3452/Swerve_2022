// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperIndex;

public class UpperIndexCommand extends CommandBase {
  private final UpperIndex index;
  private boolean direction;
  private final boolean stop;

  public UpperIndexCommand(UpperIndex index, boolean direction) {
    this.index = index;
    this.direction = direction;
    addRequirements(index);
    this.stop = false;
  }

  public UpperIndexCommand(UpperIndex index, boolean direction, boolean stop) {
    this.index = index;
    this.direction = direction;
    addRequirements(index);
    this.stop = stop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if(!direction){
      index.index(-.75);
    }else{
      index.index(0.75);
    }

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
