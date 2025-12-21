package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.signum;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

@Config
public class swerveModule {

    private double MAX_SERVO = 1, MAX_MOTOR = 1;

    private DcMotorEx motor;
    private CRServoImplEx servo;
    private AnalogInput encoder;
    private PIDController rotationController;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double current = 0.0;
    private boolean inveresed = false;
    double lastMotorPower;

    private double k_static = 0.02, p = 0.8, i = 0.0, d = 0.002;
    private double offset;

    public swerveModule(DcMotorEx m, CRServoImplEx s, AnalogInput e, Double o, Boolean inv) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = s;
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        encoder = e;
        offset = o;
        inveresed = inv;

        rotationController = new PIDController(p, i, d);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(double wa, double ws) {
        rotationController.setPID(p, i, d);

        double target = normalizeRadians(wa), current = normalizeRadians(getCurrentRotation());

        double error = normalizeRadians(target - current);

        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }
        error = normalizeRadians(target - current);

        if (Math.abs(error) < 0.05) {
            error = 0;
        }

        double power = Range.clip(rotationController.calculate(error, 0), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;
        servo.setPower(power + (Math.abs(error) > 0.02 ? signum(power) * k_static : 0));

        double motorPower = ws;
        if (wheelFlipped) {
            motorPower = -motorPower;
        }
        else {
            motorPower = motorPower;
        }

        motor.setPower(motorPower);
        lastMotorPower = motorPower;
    }

    public double getCurrentRotation() {
        double encoderVoltage = MathUtils.clamp(encoder.getVoltage(), 0, 3.3);
        double pos = 0;

        if (!inveresed) {
            pos = (encoderVoltage/3.3) * 2*Math.PI - offset;
        }
        else {
            pos = (1 - encoderVoltage/3.3) * 2*Math.PI - offset;
        }

        return normalizeRadians(pos);
    }

    public double getTargetRotation() {
        return target;
    }

    public void setOffset(double o) {
        offset = o;
    }

    public void setInverse(boolean inv) {
        inveresed = inv;
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public String getTele() {
        return String.format(Locale.ENGLISH, "Motor Power %.2f \nWheel Flipped %b \nTarget Position %.2f \nCurrent Position %.2f", lastMotorPower, wheelFlipped, target, current);
    }
}