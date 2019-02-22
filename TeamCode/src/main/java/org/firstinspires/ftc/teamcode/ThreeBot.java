package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
    public DcMotor motorShovel;
//    public Servo shovelServo;

    public int shovelTarget;

    public Rev2mDistanceSensor distanceSensor;
    public ColorSensor lightSensor;
//    public RevTouchSensor touchLifted;
    public RevTouchSensor touchDescended;

    public CRServo lifter;

    public ThreeBot(HardwareMap OrigMap, OpMode currOpMode)
    {

        // Get reference to the Op Mode's hardware Map.
        opMode = currOpMode;
        hardwareMap = OrigMap;

        // Get references to motors.
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorCenter = hardwareMap.get(DcMotor.class, "motorCenter");
        motorShovel = hardwareMap.get(DcMotor.class, "motorShovel");

        // Set the direction of each motor.
        motorLeft.setDirection(Direction.REVERSE);
        motorRight.setDirection(Direction.FORWARD);
        motorCenter.setDirection(Direction.FORWARD);
        motorShovel.setDirection(Direction.FORWARD);

        // Set all motors to brake to keep positions set.
        motorLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorCenter.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorShovel.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorShovel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShovel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter = hardwareMap.get(CRServo.class, "lifter");
        lifter.setDirection(Direction.REVERSE);
//        shovelServo = hardwareMap.get(Servo.class, "shovelServo");
//        shovelServo.setDirection(Servo.Direction.REVERSE);

        // get references to the limit switches
        lightSensor =  (hardwareMap.get(ColorSensor.class, "lightSensor"));
        distanceSensor =  (Rev2mDistanceSensor) (hardwareMap.get(DistanceSensor.class, "distanceSensor"));
    }

    public void driveBot(double driveLeft, double driveRight, double driveCenter) {

        motorLeft.setPower(driveLeft);
        motorRight.setPower(driveRight);
        motorCenter.setPower(driveCenter);
    }

    public void raiseLifter(double power) {
        lifter.setPower(power);
    }

    public void lowerLifter(double power) {
        lifter.setPower(-power);
    }

    public void stopLifter() {
        lifter.setPower(0);
    }

    public boolean isLifted()
    {
        if(lightSensor.red() > 80)
            return true;
        else
            return false;
    }

    public boolean isDescended() {
        if(distanceSensor.getDistance(DistanceUnit.INCH) < 3.45)
            return true;
        else
            return false;
    }


    public void autoHarvesterIn(double power, int distance)
    {
        motorShovel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShovel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShovel.setTargetPosition(distance);
        motorShovel.setPower(power);
    }
    public void autoHarvesterOut(double power, int distance)
    {
        motorShovel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShovel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShovel.setTargetPosition(distance);
        motorShovel.setPower(power);
    }

    public void harvesterIn(double power)
    {
        motorShovel.setPower(power);
    }
    public void harvesterOut(double power)
    {
        motorShovel.setPower(-power);
    }


    public void activateShovel(Telemetry telemetry)
    {
//        motorShovel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorShovel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorShovel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        telemetry.addData("SHOVEL STATUS:", "Run to Position");
//        telemetry.update();
    }
    public void raiseShovel(Telemetry telemetry)
    {
//        shovelTarget = motorShovel.getCurrentPosition() - 13;
//        motorShovel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorShovel.setTargetPosition(shovelTarget);
//        motorShovel.setPower(.4);
//        telemetry.addData("Shovel Position:", motorShovel.getCurrentPosition());
//        telemetry.addData("Shovel Target:", shovelTarget);
    }
    public void lowerShovel(Telemetry telemetry)
    {
//        shovelTarget = motorShovel.getCurrentPosition() + 16;
//        motorShovel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorShovel.setTargetPosition(shovelTarget);
//        motorShovel.setPower(-.4);
//        telemetry.addData("Shovel Position:", motorShovel.getCurrentPosition());
//        telemetry.addData("Shovel Target:", shovelTarget);
    }

    public void openShovel()
    {
        //shovelServo.setPosition(0);
    }
    public void closeShovel()
    {
        //shovelServo.setPosition(.45);
    }
    public void servoTelemetry(Telemetry telemetry)
    {
        //telemetry.addData("shovelServo Position", shovelServo.getPosition());
    }

}
