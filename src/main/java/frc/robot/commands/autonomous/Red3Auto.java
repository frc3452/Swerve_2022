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
import frc.robot.commands.IntakeAndLowShoot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.LowShooterCommand;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red3Auto extends SequentialCommandGroup {

  public Red3Auto(SwerveDrive swerve, Intake intake, UpperIndex index, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    //  addCommands(new FooCommand(), new BarCommand());
   // SmartDashboard.putString("autonomous", "IamRunning");
   // addCommands(new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0)))));
    // var move_to_ball_1 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.FIRST_BALL_X,C_FIELD_POSITIONS.FIRST_BALL_Y), new Rotation2d(-3.1415/2)));
    // var move_to_ball_2 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.SECOND_BALL_X,C_FIELD_POSITIONS.SECOND_BALL_Y), new Rotation2d(0.0)));
    // var move_to_ball_3 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.THIRD_BALL_X,C_FIELD_POSITIONS.THIRD_BALL_Y), new Rotation2d(0.0)));
    // var move_to_ball_4 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.FOURTH_BALL_X,C_FIELD_POSITIONS.FOURTH_BALL_Y), new Rotation2d(0.0)));
    // var shoot_1 = new ParallelDeadlineGroup(new WaitCommand(1.5), new IntakeAndShoot(intake, index, shooter)).with
    var setStart = new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(8.2,6), new Rotation2d(0))));
    var shooting = new InstantCommand(() ->new LowShooterCommand(shooter, true, true));
    var indexer = new ParallelDeadlineGroup(new IntakeAndLowShoot(intake, index, shooter));
    var Backup = new MoveToPosition(swerve, new Pose2d((new Translation2d(8.2,7.2)), new Rotation2d(0)));
    ///addCommands(move_to_position);
    addCommands(setStart, indexer);  
    
    //, move_to_ball_2, shoot, move_to_ball_3, shoot, move_to_ball_4, shoot);
  }
  
}
