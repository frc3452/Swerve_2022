package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeActuation;

public class IntakeActuationCommand extends CommandBase {

    private final IntakeActuation intakeActuation;
    private boolean direction;

    public IntakeActuationCommand(IntakeActuation intakeActuation, boolean direction) {
        this.intakeActuation = intakeActuation;
        this.direction = direction;
        addRequirements(intakeActuation);
    }

    @Override
    public void initialize() {}

    public void execute() {
        if(!direction) {
            intakeActuation.actuate(.75);
        }else {
            intakeActuation.actuate(-0.65);
        }
        
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
