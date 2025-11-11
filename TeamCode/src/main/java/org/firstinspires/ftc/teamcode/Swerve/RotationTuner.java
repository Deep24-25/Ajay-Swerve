package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;

@Config
@TeleOp
public class RotationTuner extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private CRServoImplEx FLS = null;
    private CRServoImplEx FRS = null;
    private CRServoImplEx BLS = null;
    private CRServoImplEx BRS = null;
    private CRServoImplEx[] servos;

    private AnalogInput FLE = null;
    private AnalogInput FRE = null;
    private AnalogInput BLE = null;
    private AnalogInput BRE = null;

    private AbsoluteAnalogEncoder AFLE, AFRE, ABLE, ABRE;
    private AbsoluteAnalogEncoder[] AbsoluteAnalogEncoders;

    public static double target;
    private double current;
    private double error;

    public static double P, I, D, K_Static;
    private PIDController rotationController;
    private double power;

    public static boolean inversed = false;
    public static double zero;

    private double x, y, heading;
    private static double trackwidth = 13.0, wheelbase = 13.0;
    private double wa[] = new double[4];

    public static boolean normalize = false;
    private boolean flipped;

    public static int i;
    public static boolean gamepad;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FLS = hardwareMap.get(CRServoImplEx.class, "FLS");
        FRS = hardwareMap.get(CRServoImplEx.class, "FRS");
        BLS = hardwareMap.get(CRServoImplEx.class, "BLS");
        BRS = hardwareMap.get(CRServoImplEx.class, "BRS");
        servos = new CRServoImplEx[]{FLS,FRS,BLS,BRS};

        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");
        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");

        AFLE = new AbsoluteAnalogEncoder(FLE, 3.3);
        AFRE = new AbsoluteAnalogEncoder(FRE, 3.3);
        ABLE = new AbsoluteAnalogEncoder(BLE, 3.3);
        ABRE = new AbsoluteAnalogEncoder(BRE, 3.3);
        AbsoluteAnalogEncoders = new AbsoluteAnalogEncoder[]{AFLE,AFRE,ABLE,ABRE};

        rotationController = new PIDController(P, I, D);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad) {
                x = -gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
                heading = gamepad1.left_trigger - gamepad1.right_trigger;

                double R = hypot(trackwidth, wheelbase);
                double  a = x - heading * (wheelbase / R),
                        b = x + heading * (wheelbase / R),
                        c = y - heading * (trackwidth / R),
                        d = y + heading * (trackwidth / R);

                wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
                target = wa[i];
            }
            else {
                target = target;
            }

                AbsoluteAnalogEncoders[i].zero(zero);
                AbsoluteAnalogEncoders[i].setInverted(inversed);
                current = normalizeRadians(AbsoluteAnalogEncoders[i].getCurrentPosition());

                target = normalizeRadians(target);
                error = normalizeRadians(target - current);

                if (Math.abs(error) < 0.02) {
                    error = 0.0;
                }

                if (normalize) {
                    if (Math.abs(error) > Math.PI / 2) {
                        target = normalizeRadians(target - Math.PI);
                        error = normalizeRadians(target - current);
                        flipped = true;
                    }
                    else {
                        flipped = false;
                    }
                }

                rotationController.setPID(P, I, D);
                power = rotationController.calculate(error, 0);
                servos[i].setPower(power + ((Math.abs(error) > 0.02 ? (K_Static)*signum(power) : 0)));

            telemetry.addData("target", target);
            telemetry.addData("current", current);
            telemetry.update();
        }
    }
}
