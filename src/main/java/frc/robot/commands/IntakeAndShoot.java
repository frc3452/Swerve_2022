// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpperIndex;

public class IntakeAndShoot extends CommandBase {

  private Intake intake;
  private UpperIndex index;
  private Shooter shooter;
  private boolean direction;

  /** Creates a new IntakeAndShoot. */
  public IntakeAndShoot(Intake intake, UpperIndex index, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // System.out.println("nah");
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   
    // var i = new IntakeCommand(intake, true);
    var u = new UpperIndexCommand(index, true);
    var s = new ShooterCommand(shooter, true, true);
      new WaitCommand(3).raceWith(s.alongWith(wait(1000) andThen(u.execute())));
     //   s.execute();
       // new WaitCommand(1.25);
       // u.execute();
    // i.execute();
   //  new WaitCommand(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
