package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.AbsoluteAnalogEncoder;

import java.util.Locale;

@Config
public class Module {

    private double MAX_SERVO = 1, MAX_MOTOR = 1;

    private DcMotorEx motor;
    private CRServoImplEx servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDController rotationController;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double position = 0.0;
    private boolean inverted = false;
    double lastMotorPower;

    private double k_static, p, i, d;

    public Module(DcMotorEx m, CRServoImplEx s, AbsoluteAnalogEncoder e, double P, double I, double D, double K_Static) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = s;
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        encoder = e;

        p = P; i = I; d = D; k_static = K_Static;
        rotationController = new PIDController(P, I, D);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(double Power) {
        rotationController.setPID(p, i, d);

        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);

        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        }
        else {
            wheelFlipped = false;
        }

        error = normalizeRadians(target - current);

        if (Math.abs(error) < 0.05) {
            error = 0;
        }

        double power = Range.clip(rotationController.calculate(error, 0), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;
        servo.setPower(power + (Math.abs(error) > 0.02 ? signum(power) * k_static : 0));
    }

    public void setMotorPower(double power) {

        if (wheelFlipped) {
            power *= -1;
        }

        lastMotorPower = power;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

    public double getTargetRotation() {
        return normalizeRadians(target);
    }

    public double getModuleRotation() {
        return normalizeRadians(encoder.getCurrentPosition());
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public String getTele() {
        return String.format(Locale.ENGLISH,"Motor Flipped: %b current position %.2f target position %.2f, motor power %.2f", wheelFlipped, getModuleRotation(), getTargetRotation(), lastMotorPower);
    }
}