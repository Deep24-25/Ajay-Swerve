package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import java.util.Arrays;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Geo.MathUtils;
import org.firstinspires.ftc.teamcode.Geo.Point;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

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

    //FL, BL, BR, FR
    private AbsoluteAnalogEncoder AFLE, AFRE, ABLE, ABRE;
    private AbsoluteAnalogEncoder[] Encoders;
    public static double zeros[] = new double[]{-0.2, 1.1, 3.2, 1.1};

    public static boolean inverses[] = new boolean[]{false,false,false,false};
    public static double MotorScaling[] = new double[]{1,1,1,1}; //dont make negative inverse the encoder

    GoBildaPinpointDriver odo;

    public Module frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public Module[] modules;

    public static double x, y, heading;
    private double BotHeading;
    double[] ws = new double[4];
    double[] wa = new double[4];

    private double trackwidth = 13.0; //CC distances of modules //find in CAD
    private double wheelbase = 13.0;  // trackwidth is along the width of the robot wheel base is along the length
    private double R;

    private double Xoffset, Yoffset;
    private Pose2D pos;

    private double MAX;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private SlewRateLimiter[] SlewRateLimiters;

    public static double xrate = 2.0, yrate = 2.0, headingrate = 2.0;
    public static double[] SlewRateLimits = new double[3];

    public static int i; //only used for dash to telemetry data from the different modules

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FLM = hardwareMap.get(DcMotorEx.class, "FLM"); //hardware map
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
        Encoders = new AbsoluteAnalogEncoder[]{AFLE, ABLE, ABRE, AFRE};

        AFLE.zero(zeros[0]); AFLE.setInverted(inverses[0]);
        AFRE.zero(zeros[1]); AFRE.setInverted(inverses[1]);
        ABRE.zero(zeros[2]); ABRE.setInverted(inverses[2]);
        ABLE.zero(zeros[3]); ABLE.setInverted(inverses[3]);

        frontLeftModule = new Module(FLM,FLS,AFLE,0.7,0.0,0.002,0.02);
        backLeftModule = new Module(BLM,BLS,ABLE, 0.7,0.0,0.002,0.02);
        backRightModule = new Module(BRM,BRS,ABRE,0.7,0.0,0.002,0.02);
        frontRightModule = new Module(FRM,FRS,AFRE,0.7,0.0,0.002,0.02);

        modules = new Module[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (Module m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Xoffset = 10.5; Yoffset = 1; //find in CAD
        odo.setOffsets(Xoffset, Yoffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        Pose2D startingpos = new Pose2D(DistanceUnit.CM, 0.0, 0.0, AngleUnit.RADIANS, 0.0);
        odo.setPosition(startingpos);

        odo.recalibrateIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.options) {
                odo.resetPosAndIMU();
                odo.update();
            }

            pos = odo.getPosition();
            BotHeading = -pos.getHeading(RADIANS);

            AFLE.zero(zeros[0]); AFLE.setInverted(inverses[0]);
            AFRE.zero(zeros[1]); AFRE.setInverted(inverses[1]);
            ABRE.zero(zeros[2]); ABRE.setInverted(inverses[2]);
            ABLE.zero(zeros[3]); ABLE.setInverted(inverses[3]);

            x = gamepad1.left_stick_x; //getting gamepad inputs
            y = -gamepad1.left_stick_y;
            heading = -gamepad1.right_stick_x;

            if (abs(x) < 0.02){
                x = 0;
            }

            if (abs(y) < 0.02){
                y = 0;
            }

            if (abs(heading) < 0.02){
                heading = 0;
            }

            Pose drive = new Pose((new Point(x,y).rotate(BotHeading)), heading); //creating target pose and rotating it to become field centric

            XRate.setPositiveRateLimit(xrate);
            XRate.setNegativeRateLimit(-xrate);

            YRate.setPositiveRateLimit(yrate);
            YRate.setNegativeRateLimit(-yrate);

            HeadingRate.setPositiveRateLimit(headingrate);
            HeadingRate.setNegativeRateLimit(-headingrate);

            drive.x = XRate.calculate(drive.x); //calculating wheel speeds and angles
            drive.y = YRate.calculate(drive.y);
            drive.heading = HeadingRate.calculate(drive.heading);

            double R = hypot(wheelbase, trackwidth);
            double  a = drive.x - drive.heading * (wheelbase / R), //-2
                    b = drive.x + drive.heading * (wheelbase / R),
                    c = drive.y - drive.heading * (trackwidth / R),
                    d = drive.y + drive.heading * (trackwidth / R);

            //FL, BL, BR, FR
            ws = new double[]{hypot(b,c), hypot(a, d), hypot(b, d), hypot(a, c)};
            wa = new double[]{atan2(b,c), atan2(a,d), atan2(b,d), atan2(a,c)};

            if (x == 0 && y == 0 && heading == 0) {
                ws = new double[]{0,0,0,0};
                wa = new double[]{atan2(1,-1), atan2(-1,1), atan2(1,1), atan2(-1,-1)};
            }

            for (int i = 0; i < 4; i++) {
                Module m = modules[i]; //updating each module
                ws[i] = ws[i] * MotorScaling[i];
                m.setTargetRotation(MathUtils.norm(wa[i]));
                m.update(ws[i]);
                odo.update();
            }

            telemetry.addData("wheel angle", Arrays.toString(wa));
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", heading);
            telemetry.addData("BotHeading", BotHeading);
            telemetry.addData("transformed x", drive.x);
            telemetry.addData("transformed y", drive.y);
            telemetry.addData("wheel speed", Arrays.toString(ws));
            telemetry.addData("front left module", frontLeftModule.getTele());
            telemetry.addData("front right module", frontRightModule.getTele());
            telemetry.addData("back right module", backRightModule.getTele());
            telemetry.addData("back left module", backLeftModule.getTele());

            telemetry.update();
        }

    }

}