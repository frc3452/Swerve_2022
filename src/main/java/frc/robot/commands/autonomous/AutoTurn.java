// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.MoveToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTurn extends SequentialCommandGroup {

  private InstantCommand stop(SwerveDrive swerve) {
    return new InstantCommand(() -> swerve.stop());
  }
  /** Creates a new AutoTurn. */
  int direction;
  public AutoTurn(SwerveDrive swerve, int direction) {
    this.direction = direction;
    var setStart = new InstantCommand(() -> swerve.resetPosition(new Pose2d(new Translation2d(0,0), new Rotation2d(-1.57))));
    var stop = stop(swerve);

    
    switch (direction) {
      case 0:
        var turnback = new MoveToPosition(swerve, new Pose2d((new Translation2d(0,0)), new Rotation2d(3.14)));
        addCommands(setStart, turnback, stop);
        break;
      case 1:
        var turnfront = new MoveToPosition(swerve, new Pose2d((new Translation2d(0,0)), new Rotation2d(-3.14)));
        addCommands(setStart, turnfront, stop);
        break;
      case 2:
        var turnright = new MoveToPosition(swerve, new Pose2d((new Translation2d(0,0)), new Rotation2d(-1.57)));
        addCommands(setStart, turnright, stop);
        break;
      case 3:
        var turnleft = new MoveToPosition(swerve, new Pose2d((new Translation2d(0,0)), new Rotation2d(1.57)));
        addCommands(setStart, turnleft, stop);
        break;
    }
    
    
  }
}
