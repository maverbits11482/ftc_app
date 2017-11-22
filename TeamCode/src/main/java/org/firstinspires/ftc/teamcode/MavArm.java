package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.RobotLog;

import static android.R.attr.left;
import static android.R.attr.x;
import static org.firstinspires.ftc.teamcode.ButtonLogic.changeTime;

/**
 * Created by Henry on 11/22/2017.
 */

@TeleOp
public class MavArm extends OpMode
{

    boolean rBumpPressed;
    boolean lBumpPressed;
    Servo leftClaw;
    Servo rightClaw;

    double startTime;
    double currTime;
    double elapsedTime;

    public static final double changeTime = .025;
    double distance = .3;

    double startPos = 0;
    double endPos = 1;
    double totTime = 1;
    double velocity;

    @Override
    public void init()
    {

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

        rightClaw.setDirection(Direction.REVERSE);

        rBumpPressed = false;
        lBumpPressed = false;

        startTime = this.getRuntime();

        velocity = (endPos - startPos) / totTime;

        leftClaw.setPosition(distance);
        rightClaw.setPosition(distance);

    }

    @Override
    public void loop()
    {

        currTime = this.getRuntime();

        elapsedTime = currTime - startTime;

        if(gamepad1.right_bumper)
        {
            if(rBumpPressed == false) {
                rBumpPressed = true;

                startTime = currTime;


                RobotLog.vv("Right Bumper", "Bumper Just Pressed");
            }
            else
            {
                if(elapsedTime >= changeTime)
                {
                    if(distance < 1)
                    {
                        distance = distance + (elapsedTime * velocity);

                        startTime = currTime;
                    }
                    else
                    {

                    }
                }
                else
                {

                }

            }

        }
        else if(!gamepad1.right_bumper)
        {
            if(rBumpPressed == true)
            {
                rBumpPressed = false;

                RobotLog.vv("Right Bumper", "Bumper just Released");
            }
            else
            {

            }
        }

        if(gamepad1.left_bumper)
        {
            if(lBumpPressed == false) {
                lBumpPressed = true;

                startTime = currTime;


                RobotLog.vv("Left Bumper", "Bumper Just Pressed");
            }
            else
            {
                if(elapsedTime >= changeTime)
                {
                    if(distance > .3)
                    {
                        distance = distance - (elapsedTime * velocity);

                        startTime = currTime;
                    }
                    else
                    {

                    }
                }
                else
                {

                }

            }

        }
        else if(!gamepad1.left_bumper)
        {
            if(lBumpPressed == true)
            {
                lBumpPressed = false;

                RobotLog.vv("Left Bumper", "Bumper just Released");
            }
            else
            {

            }
        }

        leftClaw.setPosition(distance);
        rightClaw.setPosition(distance);

    }
}
