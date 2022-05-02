// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.C_FIELD_POSITIONS;
import frc.robot.commands.IntakeAndShoot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.LowShooterCommand;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red2Auto extends SequentialCommandGroup {

  public Red2Auto(SwerveDrive swerve, Intake intake, UpperIndex index, Shooter shooter) {
    var setStart = new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(/*10.2,4*/7.8,4), new Rotation2d(0))));
    var shooting = new ShooterCommand(shooter, true);
    var indexer = new IntakeAndShoot(intake, index);
    var Backup = new MoveToPosition(swerve, new Pose2d((new Translation2d(/*11.4,4*/6.6,4)), new Rotation2d(0)));
    ///addCommands(move_to_position);
    addCommands(setStart, Backup, 
    new ParallelCommandGroup(shooting.withTimeout(6), new SequentialCommandGroup(new WaitCommand(2), indexer.withTimeout(4))));
  }
}
