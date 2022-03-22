package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class DriveSwerveIndefintely extends CommandBase {

    private final SwerveDrive swerve;

    private final double x, y, theta;

    public DriveSwerveIndefintely(SwerveDrive swerve, double x, double y, double theta) {
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.theta = theta;
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {      
        SwerveDrive.isFieldOriented = false;
    }

    @Override
    public void execute() {
        swerve.drive(x, y, theta);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
        //sysout
    }
}
