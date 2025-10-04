package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Geo.MathUtils;
import org.firstinspires.ftc.teamcode.Geo.Point;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Module;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;

import java.util.Arrays;

@TeleOp
public class SwerveTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx FLM = null;
    private DcMotorEx FRM = null;
    private DcMotorEx BLM = null;
    private DcMotorEx BRM = null;

    private CRServoImplEx FLS = null;
    private CRServoImplEx FRS = null;
    private CRServoImplEx BLS = null;
    private CRServoImplEx BRS = null;

    private AnalogInput FLE = null;
    private AnalogInput FRE = null;
    private AnalogInput BLE = null;
    private AnalogInput BRE = null;

    private AbsoluteAnalogEncoder AFLE, AFRE, ABLE, ABRE;

    public Module frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public Module[] modules;

    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;

    private double BotHeading;
    double[] ws = new double[4];
    double[] wa = new double[4];
    double[] cwa = new double[4];

    private double trackwidth = 9.0;
    private double wheelbase = 9.0;
    private double R;

    private double frp, flp, brp, blp, MAX;

    @Override
    public void runOpMode() throws InterruptedException{

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

        AFLE = new AbsoluteAnalogEncoder(FLE, 3.3);
        AFRE = new AbsoluteAnalogEncoder(FRE, 3.3);
        ABLE = new AbsoluteAnalogEncoder(BLE, 3.3);
        ABRE = new AbsoluteAnalogEncoder(BRE, 3.3);

        frontLeftModule = new Module(FLM,FLS,AFLE);
        frontRightModule = new Module(FRM,FRS,AFRE);
        backLeftModule = new Module(BLM,BLS,ABLE);
        backRightModule = new Module(BRM,BRS,ABRE);

        modules = new Module[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (Module m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            heading = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double BotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (abs(x) < 0.02){
                x = 0;
            }
            if (abs(y) < 0.02){
                y = 0;
            }
            if (abs(heading) < 0.02){
                heading = 0;
            }

            Pose drive = new Pose((new Point(x,y)), heading);


            double R = hypot(wheelbase, trackwidth);

            double a = x - heading * (wheelbase / R),
                    b = x + heading * (wheelbase / R),
                    c = y - heading * (trackwidth / R),
                    d = y + heading * (trackwidth / R);

            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};

            MAX = MathUtils.max(ws);

            for (int i = 0; i < 4; i++) {
                Module m = modules[i];
                if (Math.abs(MAX) > 1) ws[i] /= MAX;
                m.setMotorPower(Math.abs(ws[i]));
                m.setTargetRotation(MathUtils.norm(wa[i]));
                m.update();
            }

            telemetry.addData("calced wheel angles", Arrays.toString(wa));
            telemetry.addData("FL Angle", frontLeftModule.getModuleRotation());
            telemetry.addData("FR Angle", frontRightModule.getModuleRotation());
            telemetry.addData("BL Angle", backLeftModule.getModuleRotation());
            telemetry.addData("BR Angle", backRightModule.getModuleRotation());
            telemetry.addData("FLE Voltage", FLE.getVoltage());
            telemetry.addData("FRE Voltage", FRE.getVoltage());
            telemetry.addData("BLE Voltage", BLE.getVoltage());
            telemetry.addData("BRE Voltage", BRE.getVoltage());
            telemetry.addData("calced wheel speeds", Arrays.toString(ws));
            telemetry.update();

            /* frp = hypot(b,c);
            flp = hypot(b,d);
            brp = hypot(a,d);
            blp = hypot(a,c);

            MAX = MathUtils.max(frp, flp, brp, blp);

            if(MAX > 1) {
                frp /= MAX;
                flp /= MAX;
                brp /= MAX;
                blp /= MAX;
            } */

        }

    }

}
