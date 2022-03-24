package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeActuation;

public class IntakeActuationCommand extends CommandBase {

    private final IntakeActuation intakeActuation;

    public IntakeActuationCommand(IntakeActuation intakeActuation) {
        this.intakeActuation = intakeActuation;
        addRequirements(intakeActuation);
    }

    @Override
    public void initialize() {}

    public void execute() {
        intakeActuation.actuate(.001);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeActuation.stop();
    }

    
}
