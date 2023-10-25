package org.firstinspires.ftc.teamcode.drive.Sensors;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class ContinousAnalogAxon {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange, position;
    private boolean inverted;

    public ContinousAnalogAxon(AnalogInput enc){
        encoder = enc;
        analogRange = DEFAULT_RANGE;
        offset = 0;
        inverted = false;
    }
    public ContinousAnalogAxon zero(double off){
        offset = off;
        return this;
    }

    private double pastPosition = 0;
    public double getCurrentPosition() {
        double pos = encoder.getVoltage() * analogRange * 360;
        double dif = pos-pastPosition;
        position += dif;
        pastPosition = pos;
        return position;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }


    public double getVoltage(){
        return encoder.getVoltage();
    }
}
