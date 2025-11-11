package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
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

@Config
@TeleOp
public class PinPointTuner extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriver odo;

    Pose2D pos;
    double x, y, heading;
    public static double Xoffset, Yoffset;


    @Override
    public void runOpMode() throws InterruptedException{
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(Xoffset, Yoffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startingpos = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0);
        odo.setPosition(startingpos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            odo.update();

            pos = odo.getPosition();
            heading = pos.getHeading(RADIANS);
            x = pos.getX(DistanceUnit.INCH); y = pos.getY(DistanceUnit.INCH);

            telemetry.addData("x pos", x);
            telemetry.addData("y pos", y);
            telemetry.addData("heading", heading);

        }
    }
}
