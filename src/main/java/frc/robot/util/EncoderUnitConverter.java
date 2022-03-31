package frc.robot.util;

public class EncoderUnitConverter {
    private final Scale position_tick_to_unit;
    public Scale position_unit_to_tick;
    private final Scale velocity_unit_to_tick;
    private final Scale velocity_tick_to_unit;
    private final Scale velocity_unit_to_control_unit;
    private final Scale position_unit_to_control_unit;

    /**
     * @param gearing_from_encoder_to_wheel If wheel (arm, sprocket) rotates 3 times for every encoder one encoder rotation, enter 3
     * @param ticks_per_rotation            How many units are in one rotation of the encoder?
     * @param time_unit_in_seconds          Convert time unit to seconds. (If encoder reports RPM, enter 60. If encoder reports 100m/s, enter 0.1)
     */
    public EncoderUnitConverter(double gearing_from_encoder_to_wheel, double ticks_per_rotation, double time_unit_in_seconds, double control_unit_max, double radius) {
        //motor ticks -> motor rotations
        //motor rotations -> shaft rotations
        //shaft rotations -> shaft radians
        //((tick / ticks_per_rotation) / gearing_from_encoder_to_wheel) * (2.0 * Math.PI);
        position_tick_to_unit = new Scale((Math.PI * 2.0) / (ticks_per_rotation * gearing_from_encoder_to_wheel));
        //shaft radians -> shaft rotations
        //shaft rotations -> motor rotations
        //motor rotations -> motor ticks
        //(unit / (2.0 * Math.PI)) * gearing_from_encoder_to_wheel * ticks_per_rotation;
        position_unit_to_tick = new Scale((gearing_from_encoder_to_wheel * ticks_per_rotation) / (2.0 * Math.PI));
        velocity_unit_to_tick = Scale.from(time_unit_in_seconds, position_tick_to_unit);
        velocity_tick_to_unit = Scale.from(1.0 / time_unit_in_seconds, position_unit_to_tick);

        if (!Double.isNaN(radius)) {
            position_tick_to_unit.addScale(radius);
            position_unit_to_tick.addScale(1.0 / radius);
            velocity_tick_to_unit.addScale(radius);
            velocity_unit_to_tick.addScale(1.0 / radius);
        }

        position_unit_to_control_unit = Scale.from(control_unit_max / 12.0, 1 / position_unit_to_tick.getScale());
        velocity_unit_to_control_unit = Scale.from(control_unit_max / 12.0, 1 / velocity_unit_to_tick.getScale());
    }

    public synchronized double position_tick_to_unit(double position_tick) {
        return position_tick_to_unit.scale(position_tick);
    }

    public synchronized double velocity_tick_to_unit(double velocity_tick) {
        return velocity_tick_to_unit.scale(velocity_tick);
    }

    public synchronized double position_unit_to_tick(double position_unit) {
        return position_unit_to_tick.scale(position_unit);
    }

    public synchronized double velocity_unit_to_tick(double velocity_unit) {
        return velocity_unit_to_tick.scale(velocity_unit);
    }

    public synchronized double position_unit_to_control_unit(double position_unit) {
        return position_unit_to_control_unit.scale(position_unit);
    }

    public synchronized double velocity_unit_to_control_unit(double velocity_unit) {
        return velocity_unit_to_control_unit.scale(velocity_unit);
    }

    public synchronized double convertControlUnit(boolean velocity, double unit) {
        if (velocity) {
            return velocity_unit_to_control_unit(unit);
        }
        return position_unit_to_control_unit(unit);
    }
}
