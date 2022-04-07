// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private final Intake intake;
  private boolean shouldIntake;

  public IntakeCommand(Intake intake, boolean direction) {
    this.intake = intake;
    this.shouldIntake = direction;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if(shouldIntake){
      intake.intake(.5, .75);
    }else{
      intake.intake(-0.5, -0.75);
    }
  
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

}
