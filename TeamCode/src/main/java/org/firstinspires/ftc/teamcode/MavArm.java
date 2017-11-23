package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.RobotLog;

import static android.R.attr.left;
import static android.R.attr.x;
import static android.os.Build.VERSION_CODES.M;
import static org.firstinspires.ftc.teamcode.ButtonLogic.changeTime;
import static org.firstinspires.ftc.teamcode.MaverbitsTeleOp.ARM_POWER;

/**
 * Created by Henry on 11/22/2017.
 */

@TeleOp
public class MavArm extends OpMode
{

    boolean upPressed;
    boolean downPressed;

    DcMotor arm;

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

    double armStartPos = 0;
    double armEndPos = 1100;
    double armTime = 2;
    double armVelocity;
    double armDistance = 0;

    @Override
    public void init()
    {

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        arm = hardwareMap.dcMotor.get("arm");

        rightClaw.setDirection(Direction.REVERSE);

        arm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        arm.setMode(RunMode.RESET_ENCODERS);

        arm.setMode(RunMode.RUN_USING_ENCODER);

        rBumpPressed = false;
        lBumpPressed = false;

        upPressed = false;
        downPressed = false;

        startTime = this.getRuntime();

        velocity = (endPos - startPos) / totTime;
        armVelocity = (armEndPos - armStartPos) / armTime;

        leftClaw.setPosition(distance);
        rightClaw.setPosition(distance);

        arm.setTargetPosition((int) armDistance);

    }

    boolean bFirstIteration = true;
    @Override
    public void loop()
    {

        if(bFirstIteration) {
            bFirstIteration = false;
            arm.setMode(RunMode.RUN_TO_POSITION);
            arm.setPower(ARM_POWER);
        }
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
        if(gamepad1.dpad_up)
        {
            if(upPressed == false) {
                upPressed = true;

                startTime = currTime;

            }
            else
            {
                if(elapsedTime >= changeTime)
                {
                    if(armDistance < 1100)
                    {
                        armDistance = armDistance + (elapsedTime * armVelocity);

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
        else if(!gamepad1.dpad_up)
        {
            if(upPressed == true)
            {
                upPressed = false;

            }
            else
            {

            }
        }

        if(gamepad1.dpad_down)
        {
            if(downPressed == false) {
                downPressed = true;

                startTime = currTime;


                RobotLog.vv("Left Bumper", "Bumper Just Pressed");
            }
            else
            {
                if(elapsedTime >= changeTime)
                {
                    if(armDistance > 10)
                    {
                        armDistance = armDistance - (elapsedTime * armVelocity);

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
        else if(!gamepad1.dpad_down)
        {
            if(downPressed == true)
            {
                downPressed = false;

            }
            else
            {

            }
        }


        arm.setTargetPosition((int) armDistance);
        leftClaw.setPosition(distance);
        rightClaw.setPosition(distance);

    }
}
