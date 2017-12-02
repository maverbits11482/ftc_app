package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.R.attr.right;
import static android.R.attr.x;
import static java.lang.Runtime.getRuntime;
import static org.firstinspires.ftc.teamcode.MavBot.clawPosition.STATE_CLOSED;
import static org.firstinspires.ftc.teamcode.MavBot.clawPosition.STATE_OPEN;
import static org.firstinspires.ftc.teamcode.MavBot.clawPosition.STATE_STORAGE;
import static org.firstinspires.ftc.teamcode.MaverbitsTeleOp.ARM_POWER;


/**
 * Created by Henry on 11/11/2017.
 */

public class MavBot
{

    OpMode opMode;
    private boolean upPressed;
    private boolean downPressed;

    private boolean rBumpPressed;
    private boolean lBumpPressed;

    private boolean aPressed;
    private boolean yPressed;

    private double armStartTime;
    private double gripperStartTime;
    private double gripperElapsedTime;
    private double startTime;
    private double currTime;
    private double armElapsedTime;
    private double elapsedTime;

    public static final double changeTime = .025;
    private double distance = .3;
    private double gripperDistance = .75;

    public double clawStartPos = .3;
    public double clawEndPos = 1;
    public double clawTime = .75;
    private double velocity;

    public double gripperStartPos = .75;
    public double gripperEndPos = 0;
    public double gripperTime = 1;
    private double gripperVelocity;

    public double armStartPos = 0;
    public double armEndPos = 1100;
    public double armTime = 2;
    private double armVelocity;
    private double armDistance = 0;

    private static final double CLAW_CLOSED_POSITION = .93;
    private static final double CLAW_OPEN_POSITION = .75;
    private static final double CLAW_STORAGE_POSITION = .3;

    private HardwareMap hardwareMap;
    // Motors
    public DcMotor motorLeftFront;
    public DcMotor motorLeftRear;
    public DcMotor motorRightFront;
    public DcMotor motorRightRear;
    public DcMotor motorArm;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo gripper;


    boolean firstIteration;

    private  int armTarget;

    public enum clawPosition
    {

        STATE_STORAGE,
        STATE_OPEN,
        STATE_CLOSED;

    }

    private clawPosition currClawPosition;

    public MavBot (HardwareMap OrigMap, OpMode currOpMode)
    {

        // Get reference to the Op Mode's hardware Map.
        opMode = currOpMode;
        hardwareMap = OrigMap;
        // Get references to motors.
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftRear = hardwareMap.dcMotor.get("leftRear");
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightRear = hardwareMap.dcMotor.get("rightRear");
        motorArm = hardwareMap.dcMotor.get("arm");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

        gripper = hardwareMap.servo.get("relicClaw");

        rightClaw.setDirection(Servo.Direction.REVERSE);

        // Set the direction of each motor.
        motorLeftRear.setDirection(Direction.REVERSE);
        motorLeftFront.setDirection(Direction.REVERSE);
        motorRightFront.setDirection(Direction.FORWARD);
        motorRightRear.setDirection(Direction.FORWARD);
        motorArm.setDirection(Direction.FORWARD);

        // Set all motors to brake to keep positions set.
        motorLeftFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLeftRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRightRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motorRightRear.setMode(RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(RunMode.RUN_USING_ENCODER);

        //Reset Encoder on the arm motor with reference to the 0 position.
        motorArm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(RunMode.RUN_USING_ENCODER);


        // Set claw position to default storage state.
        currClawPosition = clawPosition.STATE_STORAGE;

        rBumpPressed = false;
        lBumpPressed = false;

        aPressed = false;
        yPressed = false;

        upPressed = false;
        downPressed = false;

        startTime = opMode.getRuntime();
        armStartTime = opMode.getRuntime();
        gripperStartTime = opMode.getRuntime();

        velocity = (clawEndPos - clawStartPos) / clawTime;
        armVelocity = (armEndPos - armStartPos) / armTime;
        gripperVelocity = (gripperStartPos - gripperEndPos) / gripperTime;

        currClawPosition = STATE_STORAGE;

        adjustClaw();

        gripper.setPosition(gripperDistance);
        leftClaw.setPosition(distance);
        rightClaw.setPosition(distance);

        motorArm.setTargetPosition((int) armDistance);

        firstIteration = true;

        // Adjust position of claw to the current state.


    }

    public void driveBot(double drivePower, double driveAngle, double turnAngle, Telemetry telemetry) {

        double forceX = -drivePower * Math.sin(driveAngle);
        double forceY = drivePower * Math.cos(driveAngle);

        double forceLR;
        double forceLF;
        double forceRR;
        double forceRF;

        // Set powers for each motor.
        forceLR = -forceX + forceY + turnAngle;
        forceLF = forceX + forceY + turnAngle;
        forceRR = forceX + forceY - turnAngle;
        forceRF = -forceX + forceY - turnAngle;

        // Clip Range so that the powers do not exceed the maximum.
        forceLR = Range.clip(forceLR, -1, 1);
        forceLF = Range.clip(forceLF, -1, 1);
        forceRR = Range.clip(forceRR, -1, 1);
        forceRF = Range.clip(forceRF, -1, 1);

        // Set power to each motor.
        motorLeftRear.setPower(forceLR);
        motorRightFront.setPower(forceRF);
        motorRightRear.setPower(forceRR);
        motorLeftFront.setPower(forceLF);

        telemetry.addData("Y Power", "%.02f", forceY);
        telemetry.addData("X Power", "%.02f", forceX);
        telemetry.addData("Left Rear Power", "%.02f", forceLR);
        telemetry.addData("Right Rear Power", "%.02f", forceRR);
        telemetry.addData("Left Front Power", "%.02f", forceLF);
        telemetry.addData("Right Front Power", "%.02f", forceRF);

    }

    public void armControl (Gamepad gamepad, Telemetry telemetry)
    {

        if(firstIteration == true) {
            firstIteration = false;
            motorArm.setMode(RunMode.RUN_TO_POSITION);
            motorArm.setPower(ARM_POWER);
        }

        telemetry.addData("Run Mode", motorArm.getMode());

        currTime = opMode.getRuntime();

        elapsedTime = currTime - startTime;
        gripperElapsedTime = currTime - gripperStartTime;
        telemetry.addData("Claw Elapsed", elapsedTime);
        telemetry.addData("Arm Elapsed", armElapsedTime);
        armElapsedTime = currTime - armStartTime;

        if(gamepad.x) {
            currClawPosition = currClawPosition.STATE_OPEN;
            adjustClaw();
        } else if (gamepad.b) {
            currClawPosition = currClawPosition.STATE_CLOSED;
            adjustClaw();
        }
        if(gamepad.right_bumper)
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
        else if(!gamepad.right_bumper)
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

        if(gamepad.left_bumper)
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
        else if(!gamepad.left_bumper)
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
        if(gamepad.dpad_up)
        {
            if(upPressed == false) {
                upPressed = true;

                armStartTime = currTime;

            }
            else
            {
                if(armElapsedTime >= changeTime)
                {
                    if(armDistance < 1100)
                    {
                        armDistance = armDistance + (armElapsedTime * armVelocity);

                        armStartTime = currTime;
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
        else if(!gamepad.dpad_up)
        {
            if(upPressed == true)
            {
                upPressed = false;

            }
            else
            {

            }
        }

        if(gamepad.dpad_down)
        {
            if(downPressed == false) {
                downPressed = true;

                armStartTime = currTime;


                RobotLog.vv("Left Bumper", "Bumper Just Pressed");
            }
            else
            {
                if(armElapsedTime >= changeTime)
                {
                    if(armDistance > 10)
                    {
                        armDistance = armDistance - (armElapsedTime * armVelocity);

                        armStartTime = currTime;
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
        else if(!gamepad.dpad_down)
        {
            if(downPressed == true)
            {
                downPressed = false;

            }
            else
            {

            }
        }

        if(gamepad.y)
        {
            if(yPressed == false) {
                yPressed = true;

                gripperStartTime = currTime;


                RobotLog.vv("Y", "Button Just Pressed");
            }
            else
            {
                if(gripperElapsedTime >= changeTime)
                {
                    if(gripperDistance < .75)
                    {
                        gripperDistance = gripperDistance + (gripperElapsedTime * gripperVelocity);

                        gripperStartTime = currTime;
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
        else if(!gamepad.y)
        {
            if(yPressed == true)
            {
                yPressed = false;

                RobotLog.vv("Y", "Button just Released");
            }
            else
            {

            }
        }

        if(gamepad.a)
        {
            if(aPressed == false) {
                aPressed = true;

                gripperStartTime = currTime;


                RobotLog.vv("A", "Button Just Pressed");
            }
            else
            {
                if(gripperElapsedTime >= changeTime)
                {
                    if(gripperDistance > 0)
                    {
                        gripperDistance = gripperDistance - (gripperElapsedTime * gripperVelocity);

                        gripperStartTime = currTime;
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
        else if(!gamepad.a)
        {

            if(aPressed == true)
            {
                aPressed = false;

                RobotLog.vv("A", "Button just Released");
            }
            else
            {

            }
        }

        telemetry.addData("ArmDistance", armDistance);

        gripper.setPosition(gripperDistance);
        motorArm.setTargetPosition((int) armDistance);
        leftClaw.setPosition(distance);
        rightClaw.setPosition(distance);
    }
    // Method for adjusting position of gripper claws.
    private void adjustClaw ()
    {

        // Changes claw position to specified state.
        switch(currClawPosition)
        {
            case STATE_STORAGE:
                distance = CLAW_STORAGE_POSITION;
                break;
            case STATE_CLOSED:
                distance = CLAW_CLOSED_POSITION;
                break;
            case STATE_OPEN:
                distance = CLAW_OPEN_POSITION;
                break;
        }

    }

}
