package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.swerve.SwerveDrive;
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {

    private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
    private final BooleanSupplier isFieldRelative;
    private final SwerveDrive swerve;

    public SwerveDriveCommand(
            SwerveDrive swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier isFieldRelative) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.isFieldRelative = isFieldRelative;
        addRequirements(swerve);
    }

    @Override

    public void initialize() {
    }

    @Override
    public void execute() {
        swerve.drive(new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble()), isFieldRelative.getAsBoolean());
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
