package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class AutonomousDrive extends LinearOpMode
{

    double turnPower = 0;
    OmniBot omniBot;
    ElapsedTime timer;
    BNO055IMU imu;
    double globalAngle;
    Orientation lastAngles;

    public void runOpMode()
    {

        omniBot = new OmniBot(hardwareMap);
        timer = new ElapsedTime();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu.initialize(parameters);

        this.waitForStart();

        timer.reset();
        resetAngle();

        while(opModeIsActive() && timer.milliseconds() < 10000)
        {
            //turnPower = getAngle()*.01;

            omniBot.driveBot(.5, Math.atan2(1, 0), turnPower, telemetry);

            telemetry.addData("Time", "%.02f", timer.milliseconds()/ 1000);
            telemetry.addData("Correction", "%.02f", turnPower);
            telemetry.update();

        }

        omniBot.driveBot(0,0,0, telemetry);

    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}
