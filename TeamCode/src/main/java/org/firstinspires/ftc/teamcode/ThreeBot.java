package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Henry on 11/11/2017.
 */

public class ThreeBot
{

    OpMode opMode;
    HardwareMap hardwareMap;

    // motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorCenter;

    boolean firstIteration;

    public ThreeBot(HardwareMap OrigMap, OpMode currOpMode)
    {

        // Get reference to the Op Mode's hardware Map.
        opMode = currOpMode;
        hardwareMap = OrigMap;

        // Get references to motors.
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorCenter = hardwareMap.get(DcMotor.class, "motorCenter");

        // Set the direction of each motor.
        motorLeft.setDirection(Direction.REVERSE);
        motorRight.setDirection(Direction.FORWARD);
        motorCenter.setDirection(Direction.FORWARD);

        // Set all motors to brake to keep positions set.
        motorLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorCenter.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        firstIteration = true;
    }

    public void driveBot(double driveLeft, double driveRight, double driveCenter) {

        motorLeft.setPower(driveLeft);
        motorRight.setPower(driveRight);
        motorCenter.setPower(driveCenter);
    }

}
