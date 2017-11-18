package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static android.R.attr.left;

/**
 * Created by Henry on 11/11/2017.
 */

@TeleOp
public class MaverbitsMechDrive extends OpMode
{

    double drivePower;
    double driveAngle;
    double turnPower;
    double leftY;
    double leftX;


    // mavBot represents our roboot.
    MavBot mavBot;

    @Override
    public void init()
    {
        // Create a new instance of MavBot.
        mavBot = new MavBot(hardwareMap);

        // Prompt user to push start button
        telemetry.addData(">>","Press start to begin");
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





        telemetry.addData("leftX", leftX);
        telemetry.addData("leftY", leftY);
        telemetry.addData("Drive angle (rad)", "%.02f", driveAngle);
        telemetry.addData("Drive angle (deg)", "%.02f", Math.toDegrees(driveAngle));



        telemetry.addData("drive Power", "%.02f", drivePower);

        telemetry.addData("turn Power", "%.02f", turnPower);

        mavBot.driveBot(drivePower, driveAngle, turnPower, telemetry);

    }
}
