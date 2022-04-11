// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeAndShoot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.ShooterCommand;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {

  public TestAuto(SwerveDrive swerve, Intake intake, UpperIndex index, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0)))));
   // var move_to_position = new MoveToPosition(swerve, new Pose2d(new Translation2d(16.0,8.0), new Rotation2d(0.0)));
    var shoot = new IntakeAndShoot(intake, index, shooter);

    ///addCommands(move_to_position);

    addCommands(shoot);
  }
}
