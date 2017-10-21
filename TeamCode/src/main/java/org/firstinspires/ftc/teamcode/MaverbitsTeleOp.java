package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/**
 * Created by Henry on 10/20/2017.
 */
@TeleOp
public class MaverbitsTeleOp extends OpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorArm;
    
    @Override
    public void init() {

        // Get references to hardware.
        motorLeft = hardwareMap.get(DcMotor.class, "left_drive");
        motorRight = hardwareMap.get(DcMotor.class, "right_drive");
        motorArm = hardwareMap.get(DcMotor.class, "arm");

        // Set direction of right motor to reverse.
        motorRight.setDirection(Direction.REVERSE);

        // Set 0 power behavior.
        motorRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    }

    /**
     * This Op mode is a driver controlled Op Mode. It uses tank drive.
     */
    @Override
    public void loop() {
        // get power from left and right joysticks.
        double leftPower = 0, rightPower = 0;
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        // scale power if needed.

        // set motor power.
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        // send info back to Driver Station using telemetry.

        telemetry.addData("Left Power", "%.02f", motorLeft.getPower());
        telemetry.addData("Right Power", "%.02f", motorRight.getPower());

    }
}
