package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwerveDrive;

public class DriveTime extends SequentialCommandGroup {

    public DriveTime(SwerveDrive swerve, ChassisSpeeds speed, boolean fieldRelative, double time) {
        addCommands(
                new DriveSwerveIndefintely(swerve, speed, fieldRelative).withTimeout(time));
    }

    private class DriveSwerveIndefintely extends CommandBase {

        private final SwerveDrive swerve;
        private final ChassisSpeeds speed;
        private final boolean fieldRelative;


        public DriveSwerveIndefintely(SwerveDrive swerve, ChassisSpeeds speed, boolean fieldRelative) {
            this.swerve = swerve;
            this.speed = speed;
            this.fieldRelative = fieldRelative;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
        }

        @Override
        public void execute() {
            swerve.drive(speed, fieldRelative);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            swerve.stop();
        }
    }

}
