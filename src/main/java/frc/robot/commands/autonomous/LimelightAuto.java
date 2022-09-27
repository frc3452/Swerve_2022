// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToPosition;
import frc.robot.subsystems.Limelight;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAuto extends SequentialCommandGroup {
  /** Creates a new LimelightAuto. */
  public LimelightAuto(SwerveDrive swerve, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var setStart = new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(0,0), new Rotation2d(-1.57))));
    double dist = limelight.calculateDistance();
    var move = new MoveToPosition(swerve, new Pose2d((new Translation2d(0,(dist / 2))), new Rotation2d(-1.57)));
    addCommands(setStart, move);
  }
}
