package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Geo.MathUtils.norm;
import static org.firstinspires.ftc.teamcode.Geo.MathUtils.normDelta;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;

    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }
    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }
    public boolean getDirection() {
        return inverted;
    }

    public double getCurrentPosition() {
        double pos = getVoltage()/3.3 * Math.PI*2 - offset;
        return pos;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}
