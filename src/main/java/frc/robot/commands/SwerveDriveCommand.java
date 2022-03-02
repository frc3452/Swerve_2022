package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {

    private final XboxController joystickdrive;
    private final SwerveDrive swerve;


    public SwerveDriveCommand(SwerveDrive swerve, XboxController joystickdrive) {
        this.swerve = swerve;
        this.joystickdrive = joystickdrive;
        addRequirements(swerve);
    }

    @Override

    public void initialize() {
    }

    @Override
    public void execute() {
        double x1 = RobotContainer.deadband(joystickdrive.getRawAxis(0));
        double y1 = RobotContainer.deadband(-joystickdrive.getRawAxis(1));
        double x2 = RobotContainer.deadband(joystickdrive.getRawAxis(4));

        swerve.drive(x1, y1, x2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}
