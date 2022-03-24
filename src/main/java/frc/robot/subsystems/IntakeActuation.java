package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeActuation extends SubsystemBase {
    private VictorSPX intakeActuation;

    public IntakeActuation(int actuator) {
        this.intakeActuation = new VictorSPX(actuator);
    }

    public void actuate(double actuateSpeed) {
        this.intakeActuation.set(VictorSPXControlMode.PercentOutput, actuateSpeed);
    }

    public void stop() {
        this.intakeActuation.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
}
