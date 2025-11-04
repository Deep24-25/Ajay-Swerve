package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;

@Config
@TeleOp
public class PIDTuner extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private CRServoImplEx FLS = null;
    private CRServoImplEx FRS = null;
    private CRServoImplEx BLS = null;
    private CRServoImplEx BRS = null;

    private AnalogInput FLE = null;
    private AnalogInput FRE = null;
    private AnalogInput BLE = null;
    private AnalogInput BRE = null;

    private AbsoluteAnalogEncoder AFLE, AFRE, ABLE, ABRE;

    public static double target;
    private double current;
    private double error;

    public static double P, I, D, K_Static;
    private PIDController rotationController;
    private double power;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FLS = hardwareMap.get(CRServoImplEx.class, "FLS");
        FRS = hardwareMap.get(CRServoImplEx.class, "FRS");
        BLS = hardwareMap.get(CRServoImplEx.class, "BLS");
        BRS = hardwareMap.get(CRServoImplEx.class, "BRS");

        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");
        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");

        AFLE = new AbsoluteAnalogEncoder(FLE, 3.3);
        AFRE = new AbsoluteAnalogEncoder(FRE, 3.3);
        ABLE = new AbsoluteAnalogEncoder(BLE, 3.3);
        ABRE = new AbsoluteAnalogEncoder(BRE, 3.3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        target = normalizeRadians(target);

        current = normalizeRadians(AFLE.getCurrentPosition());
        //current = normalizeRadians(AFRE.getCurrentPosition());
        //current = normalizeRadians(ABLE.getCurrentPosition());
        //current = normalizeRadians(ABRE.getCurrentPosition());

        error = normalizeRadians(target - current);

        rotationController.setPID(P,I,D);
        power = rotationController.calculate(0, error);
        FLS.setPower(power);
        //FRS.setPower(power);
        //BLS.setPower(power);
        //BRS.setPower(power);

        telemetry.addData("target", target);
        telemetry.addData("current", current);
        telemetry.addData("power", power);
        telemetry.addData("error", error);
    }
}
