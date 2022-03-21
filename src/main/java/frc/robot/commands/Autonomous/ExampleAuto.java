package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class ExampleAuto extends SequentialCommandGroup {

    public ExampleAuto(SwerveDrive swerve) {
        addCommands(
            // a,b,c
            // a.andThen(b).andThen(c)
            new DriveTime(swerve, 0.1, 0.2, 0.4, 5),
            new DriveTime(swerve, -0.2, 0.3, 0.1, 7),
            new DriveTime(swerve, 0.6, -0.3, 0.7, 3)
        );
    }
}
