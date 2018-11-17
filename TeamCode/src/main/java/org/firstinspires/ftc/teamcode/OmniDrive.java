package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Henry on 11/18/2017.
 */
@TeleOp
public class OmniDrive extends OpMode
{
    OmniBot omniBot;
    double drivePower;
    double driveAngle;
    double turnPower;
    double leftY;
    double leftX;


    @Override
    public void init()
    {
        omniBot = new OmniBot(hardwareMap);
    }

    @Override
    public void loop()
    {

        leftX = gamepad1.left_stick_x;
        leftY = -gamepad1.left_stick_y;

        // Find angle off y axis.
        //driveAngle = -Math.toDegrees(Math.atan2(leftX, leftY));
        driveAngle = Math.atan2(-leftX, leftY);


        // Find magnitude of power.
        drivePower = Math.hypot(-leftX, leftY);

        turnPower = gamepad1.right_stick_x;

        omniBot.driveBot(drivePower, driveAngle, turnPower, telemetry);

    }
}
