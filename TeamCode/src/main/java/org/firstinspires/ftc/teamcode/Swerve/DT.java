package org.firstinspires.ftc.teamcode.Swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Geo.MathUtils;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class DT {
    public static boolean maintainHeading;
    public Module frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public Module[] modules;

    public static double TrackWidth = 9, WheelBase = 9;
    private final double R;
    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    public static boolean MaintainHeading = false;
    public static boolean UseWheelFf = false;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;
    public static double imuAngle = 0.0;

    private boolean locked = false;

    public DT(RobotHardware robot) {
        frontLeftModule = new Module(robot.frontLeftMotor, robot.frontLeftServo, new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).zero(frontLeftOffset).setInverted(true));
        backLeftModule = new Module(robot.backLeftMotor, robot.backLeftServo, new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).zero(backLeftOffset).setInverted(true));
        backRightModule = new Module(robot.backRightMotor, robot.backRightServo, new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).zero(backRightOffset).setInverted(true));
        frontRightModule = new Module(robot.frontRightMotor, robot.frontRightServo, new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));

        modules = new Module[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (Module m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TrackWidth, WheelBase);
    }

    public void read() {
        for (Module module : modules) module.read();
    }

    public void set(@NonNull Pose pose) {
        double x = pose.x, y = pose.y, head = pose.heading;

        double a = x - head * (WheelBase / R),
                b = x + head * (WheelBase / R),
                c = y - head * (TrackWidth / R),
                d = y + head * (TrackWidth / R);

        if (locked) {
            ws = new double[]{0, 0, 0, 0};
            wa = new double[]{Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4};
        } else {
            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!MaintainHeading) wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }

        max = MathUtils.max(ws);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            Module m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]) + ((UseWheelFf) ? minPow * Math.signum(ws[i]) : 0));
            m.setTargetRotation(MathUtils.norm(wa[i]));
        }
    }

    public void updateModules() {
        for (Module m : modules) m.update();
    }

    public void setLocked(boolean locked){
        this.locked = locked;
    }

    public boolean isLocked(){
        return locked;
    }

    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }

    public double getAngle(){
        return imuAngle-imuOffset;
    }


}
