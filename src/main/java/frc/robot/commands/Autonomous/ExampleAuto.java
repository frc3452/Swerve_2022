package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.SwerveDrive;

public class ExampleAuto extends SequentialCommandGroup {

    public ExampleAuto(SwerveDrive swerve) {
        addCommands(
                // a,b,c
                // a.andThen(b).andThen(c),
                // new ShooterCommand(shooter).raceWith(new WaitCommand(5)),
                new DriveTime(swerve, 0.25, 0, 0, 2),
                new WaitCommand(2),
                new DriveTime(swerve, 0, 0.25, 0, 2),
                new WaitCommand(2),
                new DriveTime(swerve, 0, 0, 0.25, 2));
    }
}
