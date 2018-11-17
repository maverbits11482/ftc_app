package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import static android.R.attr.dropDownVerticalOffset;
import static android.R.attr.start;

/**
 * Created by Henry on 11/21/2017.
 */
@TeleOp
public class ButtonLogic extends OpMode
{

    Servo servo;

    double startTime;
    double currTime;
    double elapsedTime;

    double distance = 0;

    double startPos = 0;
    double endPos = 1;
    double totTime = 1;

    public static final double changeTime = .025;

    double velocity;

    boolean yPressed;
    boolean aPressed;
    boolean xPressed;
    boolean bPressed;

    @Override
    public void init()
    {

        servo = hardwareMap.servo.get("servo");

        velocity = (endPos - startPos) / totTime;

        startTime = this.getRuntime();

        xPressed = false;
        bPressed = false;
        yPressed = false;
        aPressed = false;

    }

    @Override
    public void loop()
    {

        currTime = this.getRuntime();
        elapsedTime = currTime - startTime;

        RobotLog.vv("HCE", "runtime = %.02f", this.getRuntime());

        if(gamepad1.x)
        {
            if(xPressed == false) {
                xPressed = true;

                startTime = currTime;


                RobotLog.vv("buttonX", "Button Just Pressed");
                telemetry.addData("State", "Just Pressed");
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
        else if(!gamepad1.x)
        {
            if(xPressed = true)
            {
                xPressed = false;

                RobotLog.vv("buttonX", "Button just Released");
                telemetry.addData("State", "Just Released");
            }
            else
            {

            }
        }

        if(gamepad1.b)
        {
            if(bPressed == false) {
                bPressed = true;

                startTime = currTime;


                RobotLog.vv("buttonX", "Button Just Pressed");
            }
            else
            {
                if(elapsedTime >= changeTime)
                {
                    if(distance > 0)
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
        else if(!gamepad1.b)
        {
            if(bPressed = true)
            {
                bPressed = false;

                RobotLog.vv("buttonB", "Button just Released");
            }
            else
            {

            }
        }

        if(gamepad1.y && yPressed == false && totTime >= .5)
        {

            yPressed = true;

            totTime = totTime - .25;

        }
        else if(!gamepad1.y && yPressed == true)
        {
            yPressed = false;
        }
        if(gamepad1.a && aPressed == false && totTime <= 1.5)
        {

            aPressed = true;

            totTime = totTime + .25;

        }
        else if(!gamepad1.a && aPressed == true)
        {
            aPressed = false;
        }

        velocity = (endPos - startPos) / totTime;

        servo.setPosition(distance);
        RobotLog.vv("Time", "Start = %.04f, Curr = %.04f, Elapsed = %.04f", startTime, currTime, elapsedTime);

        telemetry.addData("Velocity of Servo", velocity);
        telemetry.addData("Servo Position", servo.getPosition());
    }


}
