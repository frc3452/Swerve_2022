package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.UpperIndexCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.swerve.SwerveDrive;
import frc.robot.subsystems.UpperIndex;

public class ExampleAuto extends SequentialCommandGroup {

    public ExampleAuto(SwerveDrive swerve, UpperIndex index, Shooter shoot) {
        var shoot_command = new ShooterCommand(shoot, false).withTimeout(1.5)
                .andThen(
                        new ParallelDeadlineGroup(new WaitCommand(2),
                                new ShooterCommand(shoot, true),
                                new UpperIndexCommand(index, true)));
        
        var drive_command = new DriveTime(swerve, -0.25, 0, 0, 2.5).andThen(
                new WaitCommand(5.5));

        addCommands(
                shoot_command,
                new WaitCommand(0.25),
                drive_command);
    }


}

