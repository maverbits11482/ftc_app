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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.R.attr.right;
import static android.R.attr.x;



/**
 * Created by Henry on 11/11/2017.
 */

public class MavBot
{

    private HardwareMap hardwareMap;
    // Motors
    public DcMotor motorLeftFront;
    public DcMotor motorLeftRear;
    public DcMotor motorRightFront;
    public DcMotor motorRightRear;
    public DcMotor motorArm;
    public Servo leftClaw;
    public Servo rightClaw;

    private  int armTarget;

    private enum clawPosition
    {

        STATE_STORAGE,
        STATE_OPEN,
        STATE_CLOSED;

    }

    private clawPosition currClawPosition;

    public MavBot (HardwareMap OrigMap)
    {

        // Get reference to the Op Mode's hardware Map.
        hardwareMap = OrigMap;
        // Get references to motors.
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftRear = hardwareMap.dcMotor.get("leftRear");
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightRear = hardwareMap.dcMotor.get("rightRear");
        motorArm = hardwareMap.dcMotor.get("arm");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

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
        motorArm.setMode(RunMode.RUN_TO_POSITION);

        // Set claw position to default storage state.
        currClawPosition = clawPosition.STATE_STORAGE;

        // Adjust position of claw to the current state.
        adjustClaw();

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
        forceLF = forceX + forceY - turnAngle;
        forceRR = forceX + forceY + turnAngle;
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

    public void armControl (Gamepad gamepad)
    {



    }
    // Method for adjusting position of gripper claws.
    private void adjustClaw ()
    {

        // Changes claw position to specified state.
        switch(currClawPosition)
        {
            case STATE_STORAGE:
                leftClaw.setPosition(.2);
                rightClaw.setPosition(.2);
                break;
            case STATE_CLOSED:
                leftClaw.setPosition(.93);
                rightClaw.setPosition(.93);
                break;
            case STATE_OPEN:
                leftClaw.setPosition(.75);
                rightClaw.setPosition(.75);
                break;
        }

    }

}
