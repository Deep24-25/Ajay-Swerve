package org.firstinspires.ftc.teamcode.Swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Geo.MathUtils;
import org.firstinspires.ftc.teamcode.Geo.Pose;
import org.firstinspires.ftc.teamcode.Hardware.robotHardware;

import java.util.Locale;

@Config
public class swerveDrivetrain {

    public swerveModule frontLeftModule, frontRightModule, backRightModule, backLeftModule;
    public swerveModule[] modules;

    private double[] ws = new double[4];
    private double MotorScaling[] = new double[]{1,1,1,1};
    private double max;

    double[] wa = new double[4];
    private double offsets[] = new double[]{-0.2, 1.1, 3.2, 0.3};
    private boolean inverses[] = new boolean[]{false,false,false,false};

    private double trackwidth = 13.0; //CC distances of modules //find in CAD
    private double wheelbase = 13.0;  // trackwidth is along the width of the robot wheel base is along the length
    private double R;

    private boolean locked = false;

    public swerveDrivetrain(robotHardware robot) {
        frontLeftModule = new swerveModule(robot.FLM, robot.FLS, robot.FLE, offsets[0], false);
        frontRightModule = new swerveModule(robot.FRM, robot.FRS, robot.FRE, offsets[1], false);
        backRightModule = new swerveModule(robot.BRM, robot.BRS, robot.BRE, offsets[2], false);
        backLeftModule = new swerveModule(robot.BLM, robot.BLS, robot.BLE, offsets[3], false);

        modules = new swerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (swerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        R = hypot(trackwidth, wheelbase);
    }

    public void setPose (Pose pose){
        double x = pose.x, y = pose.y, heading = pose.heading;

        if (locked) {
            ws = new double[]{0,0,0,0};
            wa = new double[]{atan2(1, -1), atan2(-1, 1), atan2(1, 1), atan2(-1, -1)};
        }
        else {
            double  a = x - heading * (wheelbase / R),
                    b = x + heading * (wheelbase / R),
                    c = y - heading * (trackwidth / R),
                    d = y + heading * (trackwidth / R);

            //FL, BL, BR, FR
            ws = new double[]{hypot(b,c), hypot(a, d), hypot(b, d), hypot(a, c)};
            wa = new double[]{atan2(b,c), atan2(a,d), atan2(b,d), atan2(a,c)};
        }

        max = MathUtils.max(ws);
    }

    public void updateModules() {
        for (int i = 0; i < 4; i++) {
            swerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.update(wa[i], (ws[i]*MotorScaling[i]));
        }
    }

    public void updateModule(int i) {
        swerveModule m = modules[i];
        if (Math.abs(max) > 1) ws[i] /= max;
        m.update(wa[i], ws[i]);
    }

    public void setOffsets(double[] offsets) {
        frontLeftModule.setOffset(offsets[0]);
        frontRightModule.setOffset(offsets[1]);
        backRightModule.setOffset(offsets[2]);
        backLeftModule.setOffset(offsets[3]);
    }

    public void setInverses(boolean[] inverses) {
        frontLeftModule.setInverse(inverses[0]);
        frontRightModule.setInverse(inverses[1]);
        backRightModule.setInverse(inverses[2]);
        backLeftModule.setInverse(inverses[3]);
    }

    public void setMotorScaling(double[] scalars){
        for (int i = 0; i < 4; i++){
            MotorScaling[i] = scalars[i];
        }
    }

    public void setLocked(Boolean locked)  {
        this.locked = locked;
    }

    public boolean getLocked() {
        return  locked;
    }

    public String getTele(){
        return String.format(Locale.ENGLISH, "Front Left Module %s \nFront Right Module %s \nBack Right Module %s \nBack Left Module %s",frontLeftModule.getTele(), frontRightModule.getTele(), backRightModule.getTele(), backLeftModule.getTele());
    }
}
