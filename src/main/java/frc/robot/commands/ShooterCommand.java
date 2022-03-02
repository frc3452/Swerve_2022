// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
  private final XboxController joystick;
  private final Shooter shooter;

  public ShooterCommand(Shooter shooter, XboxController joystickcontrol) {
    this.shooter = shooter;
    this.joystick = joystickcontrol;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

 // Called every time the scheduler runs while the command is scheduled.
 public void execute() {
   double leftDecrease = RobotContainer.deadband(joystick.getRawAxis(2));
   double rightDecrease = RobotContainer.deadband(joystick.getRawAxis(3));

   shooter.shooter(leftDecrease, rightDecrease);
 }

 @Override
public void end(boolean interrupted) {
  shooter.stop();
}

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
}
