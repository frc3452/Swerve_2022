package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrive;

public class DriveTime extends SequentialCommandGroup {

    public DriveTime(SwerveDrive swerve, double x, double y, double theta, double time) {
        addCommands(
            new DriveSwerveIndefintely(swerve, x, y, theta)
            .raceWith(new WaitCommand(time))
            );
    }

}
