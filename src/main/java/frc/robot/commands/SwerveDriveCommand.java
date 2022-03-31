package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.swerve.SwerveDrive;
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {

    private final XboxController joystickdrive;
    private final BooleanSupplier isFieldRelative;
    private final SwerveDrive swerve;

    public SwerveDriveCommand(SwerveDrive swerve, XboxController joystickDrive, BooleanSupplier isFieldRelative) {
        this.swerve = swerve;
        this.joystickdrive = joystickDrive;
        this.isFieldRelative = isFieldRelative;
        addRequirements(swerve);
    }

    @Override

    public void initialize() {
    }

    @Override
    public void execute() {
        double x1 = Util.deadband(joystickdrive.getRawAxis(0));
        double y1 = Util.deadband(-joystickdrive.getRawAxis(1));
        double x2 = Util.deadband(joystickdrive.getRawAxis(4));
        //scale
        swerve.drive(new ChassisSpeeds(x1, y1, x2), isFieldRelative.getAsBoolean());
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
