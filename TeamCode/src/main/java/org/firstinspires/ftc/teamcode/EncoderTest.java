package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class EncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private AnalogInput FLE = null;

    @Override
    public void runOpMode() throws InterruptedException{

        FLE = hardwareMap.get(AnalogInput.class, "FLE");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Front Left Voltage", FLE.getVoltage());
            telemetry.update();
        }
    }

}
