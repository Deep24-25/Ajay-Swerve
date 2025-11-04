package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;

@Config
public class Module {
    public static double P = 0.2, I = 0, D = 0;
    public static double K_STATIC = 0;

    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    public static boolean MOTOR_FLIPPING = true;

    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1 / (3.5 * 1.5 * 2); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;

    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDController rotationController;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double position = 0.0;
    private boolean inverted = false;

    public Module(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        encoder = e;
        rotationController = new PIDController(P, I, D);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Module(HardwareMap hardwareMap, String mName, String sName, String eName) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(CRServo.class, sName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, eName)));
    }

    public void update() {
        rotationController.setPID(P, I, D);
        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);
        /* if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        } */

        //if (Math.abs(error) < 0.2) { error = 0.0; }

        double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        servo.setPower(power + K_STATIC);
    }

    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(encoder.getCurrentPosition() - Math.PI);
    }

    public void setMotorPower(double power) {
        power = power*1.0;
        if (wheelFlipped) power *= -1;
        if (inverted) {
            power = -1*power;
        }
        lastMotorPower = power;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public double lastMotorPower = 0;

    public double getServoPower() {
        return servo.getPower();
    }

    public double getcurrentposition() { return encoder.getCurrentPosition();}

    public void setInverted(boolean inverted) { this.inverted = inverted;}

    public double geterror() {return normalizeRadians(getTargetRotation()) - normalizeRadians(getModuleRotation());}
}