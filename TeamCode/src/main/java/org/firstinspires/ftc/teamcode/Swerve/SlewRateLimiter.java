package org.firstinspires.ftc.teamcode.Swerve;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private double PositiveRateLimit;
    private double NegativeRateLimit;
    private final ElapsedTime Timer;
    private double PrevVal;
    private double PrevTime;

    public SlewRateLimiter(double PositiveRateLimit, double NegativeRateLimit, double InitialValue){
        this.PositiveRateLimit = PositiveRateLimit;
        this.NegativeRateLimit = NegativeRateLimit;
        PrevVal = InitialValue;
        PrevTime = 0.0;
        Timer = new ElapsedTime();
    }

    public SlewRateLimiter(double RateLimit, double IntialValue) {
        this(RateLimit, -RateLimit, IntialValue);
    }

    public SlewRateLimiter(double RateLimit) {
        this(RateLimit, -RateLimit, 0);
    }

    public double calculate(double input){
        double CurrentTime = Timer.seconds();
        double ElapsedTime = CurrentTime - PrevTime;
        PrevVal += MathUtils.clamp(input - PrevVal, NegativeRateLimit * ElapsedTime, PositiveRateLimit * ElapsedTime);
        PrevTime = CurrentTime;
        return PrevVal;
    }

    public void setPositiveRateLimit(double PositiveRateLimit) {
        this.PositiveRateLimit = PositiveRateLimit;
    }

    public void setNegativeRateLimit(double NegativeRateLimit) {
        this.NegativeRateLimit = NegativeRateLimit;
    }
}
