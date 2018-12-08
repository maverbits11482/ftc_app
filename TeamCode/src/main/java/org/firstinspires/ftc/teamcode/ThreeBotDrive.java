package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Henry on 11/11/2017.
 */

@TeleOp
public class ThreeBotDrive extends OpMode
{

    ThreeBot threeBot;

    double left;
    double right;
    double drive;
    double turn;
    double max;

    double sideways;

    @Override
    public void init()
    {
        threeBot = new ThreeBot(hardwareMap, this);

        // Prompt user to push start button
        telemetry.addData(">>","Press start to begin");
    }

    @Override
    public void loop()
    {

        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.left_stick_x;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        sideways = -gamepad1.right_stick_x;

        // drive.
        threeBot.driveBot(left, right, sideways);

        telemetry.addData("left", "%.02f", threeBot.motorLeft.getPower());
        telemetry.addData("right", "%.02f", threeBot.motorLeft.getPower());
        telemetry.addData("center", "%.02f", threeBot.motorLeft.getPower());

    }
}
