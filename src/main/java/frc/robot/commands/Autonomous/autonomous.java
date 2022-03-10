package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class autonomous extends CommandBase {

    private final SwerveDrive swerve;


    public autonomous(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override

    public void initialize() {
      swerve.drive(1, 1, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}
