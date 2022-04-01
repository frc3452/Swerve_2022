package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.util.Util;

public class ClimberCommand extends CommandBase {
  private final XboxController joystick;
  private final Climber climber;

  public ClimberCommand(Climber climber, XboxController joystickcontrol) {
    this.climber = climber;
    this.joystick = joystickcontrol;
    addRequirements(climber);
  }

  @Override

  public void initialize() {
  }

  @Override
  public void execute() {
    double climberLeft = Util.deadband(joystick.getRawAxis(0));
    double climberRight = Util.deadband(joystick.getRawAxis(4));

    climber.climber(climberLeft, climberRight);

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climber.climber(0, 0);
  }
}