// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpperIndex;

public class ShooterLogic extends CommandBase {
  private Intake intake;
  private UpperIndex index;
  private Shooter shooter;
  private boolean inverted;
  private boolean constantIntake;

  /** Creates a new ShooterLogic. */
  public ShooterLogic(Intake intake,UpperIndex index, Shooter shooter, boolean inverted, boolean constantIntake) {
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
    this.inverted = inverted;
    this.constantIntake = constantIntake;
    addRequirements(intake);
    addRequirements(index);
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (inverted) {

    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
