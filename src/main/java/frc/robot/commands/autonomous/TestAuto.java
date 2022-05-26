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
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {

  public TestAuto(SwerveDrive swerve, Intake intake, UpperIndex index, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    //  addCommands(new FooCommand(), new BarCommand());
   // SmartDashboard.putString("autonomous", "IamRunning");
   // addCommands(new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0)))));
    // var move_to_ball_1 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.FIRST_BALL_X,C_FIELD_POSITIONS.FIRST_BALL_Y), new Rotation2d(-3.1415/2)));
    // var move_to_ball_2 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.SECOND_BALL_X,C_FIELD_POSITIONS.SECOND_BALL_Y), new Rotation2d(0.0)));
    // var move_to_ball_3 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.THIRD_BALL_X,C_FIELD_POSITIONS.THIRD_BALL_Y), new Rotation2d(0.0)));
    // var move_to_ball_4 = new MoveToPosition(swerve, new Pose2d(new Translation2d(C_FIELD_POSITIONS.FOURTH_BALL_X,C_FIELD_POSITIONS.FOURTH_BALL_Y), new Rotation2d(0.0)));
    // var shoot_1 = new ParallelDeadlineGroup(new WaitCommand(1.5), new IntakeAndShoot(intake, index, shooter)).with
    var setStart = new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(0,0), new Rotation2d(-1.57))));
    var stop1 = new InstantCommand(() -> swerve.stop());
    var stop2 = new InstantCommand(() -> swerve.stop());
    var stop3 = new InstantCommand(() -> swerve.stop());
    var shooting = new ShooterCommand(shooter, true);
    var shooting2 = new ShooterCommand(shooter, true);
    var shooting3 = new ShooterCommand(shooter, true);
    var indexer = new IntakeAndShoot(intake, index);
    
    var move1 = new MoveToPosition(swerve, new Pose2d((new Translation2d(-0.9,-1.3)), new Rotation2d(-1.57)));
    var move2 = new MoveToPosition(swerve, new Pose2d((new Translation2d(-0.9,-1.3)), new Rotation2d(-3.14)));
    var move3 = new MoveToPosition(swerve, new Pose2d((new Translation2d(0.0,0.0)), new Rotation2d(0)));
    addCommands(setStart, move1.raceWith(indexer.withTimeout(10)), stop1, move2.alongWith(shooting.withTimeout(5), new WaitCommand(2)));

    // , new WaitCommand(10), rotate.raceWith(shooting));
    // addCommands(setStart,move.raceWith(indexer /*shooting*/), new WaitCommand(3), rotate,new WaitCommand(4).alongWith(new WaitCommand(4).raceWith(shooting2)/*, new WaitCommand(1).andThen(new WaitCommand(3).raceWith(indexer2, shooting3))*/));
    // shooting, rotate)); 
    // new ParallelCommandGroup(shooting.withTimeout(6), new SequentialCommandGroup(new WaitCommand(2), indexer.withTimeout(4))));
    //, move_to_ball_2, shoot, move_to_ball_3, shoot, move_to_ball_4, shoot);
  }

  private InstantCommand stop(SwerveDrive swerve) {
    return new InstantCommand(() -> swerve.stop());
  }
}
