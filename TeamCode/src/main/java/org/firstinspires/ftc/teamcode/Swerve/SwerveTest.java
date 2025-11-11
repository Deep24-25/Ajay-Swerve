package org.firstinspires.ftc.teamcode.Swerve;

import java.util.Arrays;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Geo.MathUtils;
import org.firstinspires.ftc.teamcode.Geo.Point;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;

@Config
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
    public static double AFLEzero, AFREzero, ABLEzero,ABREzero;

    public Module frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public Module[] modules;

    public static double x, y, heading;
    private double BotHeading;
    double[] ws = new double[4];
    double[] wa = new double[4];

    private double trackwidth = 13.0;
    private double wheelbase = 13.0;
    private double R;

    private double MAX;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        AFLE.zero(-0.5);
        AFLE.setInverted(true);

        AFRE.zero(0.125);
        AFRE.setInverted(true);

        ABLE.zero(0.25);
        ABLE.setInverted(true);

        ABRE.zero(0);
        ABRE.setInverted(true);

        frontLeftModule = new Module(FLM,FLS,AFLE,0.5,0.0,0.002,0.02);
        frontRightModule = new Module(FRM,FRS,AFRE, 0.5,0.0,0.002,0.02);
        backLeftModule = new Module(BLM,BLS,ABLE,0.5,0.0,0.002,0.02);
        backRightModule = new Module(BRM,BRS,ABRE,0.5,0.0,0.002,0.02);

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

            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            heading = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            BotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (abs(x) < 0.02){
                x = 0;
            }
            if (abs(y) < 0.02){
                y = 0;
            }
            if (abs(heading) < 0.02){
                heading = 0;
            }

            Pose drive = new Pose((new Point(x,y).rotate(BotHeading)), heading);


            double R = hypot(wheelbase, trackwidth);

            double  a = drive.x - drive.heading * (wheelbase / R),
                    b = drive.x + drive.heading * (wheelbase / R),
                    c = drive.y - drive.heading * (trackwidth / R),
                    d = drive.y + drive.heading * (trackwidth / R);

            //top left, top right, bottom right, bottom left
            ws = new double[]{hypot(b, d), hypot(b, c), hypot(a, c), hypot(a, d)};
            wa = new double[]{atan2(b,d), atan2(b,c), atan2(a,c), atan2(a,d)};

            MAX = MathUtils.max(ws);

             for (int i = 0; i < 4; i++) {
                Module m = modules[i];
                if (Math.abs(MAX) > 1) ws[i] /= MAX;
                m.setMotorPower(Math.abs(ws[i]));
                m.setTargetRotation(MathUtils.norm(wa[i]));
                m.update();
            }

            telemetry.addData("front left target angle", Arrays.toString(wa));
            telemetry.addData("front left voltage", AFLE.getVoltage());
            telemetry.addData("front right voltage", AFRE.getVoltage());
            telemetry.addData("back left voltage", ABLE.getVoltage());
            telemetry.addData("back right voltage", ABRE.getVoltage());
            telemetry.addData("front left encoder angle", frontLeftModule.getcurrentposition());
            telemetry.addData("front left angle", frontLeftModule.getModuleRotation());
            telemetry.addData("front right angle", frontRightModule.getModuleRotation());
            telemetry.addData("back left angle", backLeftModule.getModuleRotation());
            telemetry.addData("back right angle", backRightModule.getModuleRotation());
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", heading);

            telemetry.update();
        }

    }

}
