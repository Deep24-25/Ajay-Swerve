package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Geo.Point;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import org.firstinspires.ftc.teamcode.Hardware.robotHardware;
import org.firstinspires.ftc.teamcode.Limiters.SlewRateLimiter;

@Config
@TeleOp
public class swerveTuningTele extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private robotHardware robot;
    private swerveDrivetrain swerveDrivetrain;

    public double x, y, heading;
    public double BotHeading;
    public boolean locked;

    GoBildaPinpointDriver odo;
    private double Xoffset, Yoffset;
    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    public static double xrate = 4.0, yrate = 4.0, headingrate = 3.0;

    public static double offsets[] = new double[]{-0.2, 1.1, 3.2, 1.1};
    public static boolean inverses[] = new boolean[]{false, false, false, false};
    public static double scalars[] = new double[]{1, 1, 1, 1};

    public static int i;
    public static boolean gamepad;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        robot = new robotHardware(hardwareMap);
        swerveDrivetrain = new swerveDrivetrain(robot);

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

//            if (x == 0 && y == 0 && heading == 0) {
//                locked = true;
//            }
//            else {
//                locked = false;
//            }

            Pose drive = new Pose((new Point(x,y).rotate(BotHeading)), heading);

            XRate.setPositiveRateLimit(xrate);
            XRate.setNegativeRateLimit(-xrate);

            YRate.setPositiveRateLimit(yrate);
            YRate.setNegativeRateLimit(-yrate);

            HeadingRate.setPositiveRateLimit(headingrate);
            HeadingRate.setNegativeRateLimit(-headingrate);

            drive = new Pose(XRate.calculate(drive.x), YRate.calculate(drive.y), HeadingRate.calculate(drive.heading));

            swerveDrivetrain.setOffsets(offsets);
            swerveDrivetrain.setInverses(inverses);

            swerveDrivetrain.setPose(drive);
            swerveDrivetrain.updateModules();
            swerveDrivetrain.updateModule(i);

            telemetry.addData("x", drive.x);
            telemetry.addData("y", drive.y);
            telemetry.addData("heading", drive.heading);
            telemetry.addData("tele \n", swerveDrivetrain.getTele());
            telemetry.addData("locked", locked);
            telemetry.update();
        }
    }
}
