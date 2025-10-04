/* package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.Geo.Point;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.DT;

@TeleOp

public class SwerveTest extends LinearOpMode {

    private ElapsedTime timer;
    private double loopTime = 0;

    private RobotHardware HWMap;
    private DT swerve;

    public static double position;

    private boolean pHeadingLock = true;
    private boolean lockRobotHeading = false;
    private double targetHeading;

    private double voltage = 0.0;

    private final PIDFController hController = new PIDFController(0, 0, 0, 0);

    private double getVoltage() {
        return voltage;
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        HWMap = new RobotHardware(hardwareMap, false);
        swerve = new DT(HWMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            swerve.read();
            DT.imuOffset = swerve.getAngle() + Math.PI;

            if(gamepad1.right_stick_y > 0.25){
                lockRobotHeading = true;
                targetHeading = Math.PI + swerve.imuOffset;
            }

            if(gamepad1.right_stick_y < -0.25){
                lockRobotHeading = true;
                targetHeading = swerve.imuOffset;
            }

            double turn = gamepad1.right_trigger - gamepad1.left_trigger;

            if (Math.abs(turn) > 0.002){
                lockRobotHeading = false;
            }

            double error = normalizeRadians(normalizeRadians(targetHeading)-normalizeRadians(swerve.getAngle()));
            double headingCorrection = hController.calculate(0,error) * 12.4 / getVoltage();

            if (Math.abs(headingCorrection) < 0.01){
                headingCorrection = 0;
            }

            DT.maintainHeading =
                    (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                            Math.abs(gamepad1.left_stick_y) < 0.002 &&
                            Math.abs(turn) < 0.002) &&
                            Math.abs(headingCorrection) < 0.02;

            double rotationAmount = (swerve.getAngle() - DT.imuOffset);
            Pose drive = new Pose(
                    new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                            joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                    lockRobotHeading ? headingCorrection :
                            joystickScalar(turn, 0.01)
            );

            swerve.read();
            swerve.set(drive);
            swerve.write();
            swerve.updateModules();
        }
        }

} */
