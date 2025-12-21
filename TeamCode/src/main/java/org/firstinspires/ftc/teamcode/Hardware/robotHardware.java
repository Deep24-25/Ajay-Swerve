package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class robotHardware {
    public DcMotorEx FLM;
    public CRServoImplEx FLS;
    public AnalogInput FLE;

    public DcMotorEx FRM;
    public CRServoImplEx FRS;
    public AnalogInput FRE;

    public DcMotorEx BRM;
    public CRServoImplEx BRS;
    public AnalogInput BRE;

    public DcMotorEx BLM;
    public CRServoImplEx BLS;
    public AnalogInput BLE;

    public static boolean initialized = false;

    public robotHardware(HardwareMap hardwareMap) {
        FLM = hardwareMap.get(DcMotorEx.class, "FLM");
        FRM = hardwareMap.get(DcMotorEx.class, "FRM");
        BLM = hardwareMap.get(DcMotorEx.class, "BLM");
        BRM = hardwareMap.get(DcMotorEx.class, "BRM");

        FLS = hardwareMap.get(CRServoImplEx.class, "FLS");
        FRS = hardwareMap.get(CRServoImplEx.class, "FRS");
        BLS = hardwareMap.get(CRServoImplEx.class, "BLS");
        BRS = hardwareMap.get(CRServoImplEx.class, "BRS");

        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");
        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");
    }

}
