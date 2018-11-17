package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import static android.R.attr.left;

/**
 * Created by Henry on 10/20/2017.
 */
@TeleOp
public class MaverbitsTeleOp extends OpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorArm;

    private Servo leftClaw;
    private Servo rightClaw;

    private boolean dpadUpPressed;
    private boolean dpadDownPressed;

    private enum GripperState
    {
        STATE_STORAGE,
        STATE_OPEN,
        STATE_CLOSED
    }

    private enum ArmState
    {
        STATE_H0,
        STATE_H1,
        STATE_H2,
        STATE_H3
    }

    private ArmState armState;

    private GripperState gripperState;

    // constants.
    public static final int HEIGHT_0 = 20;
    public static final int HEIGHT_1 = 380;
    public static final int HEIGHT_2 = 625;
    public static final int HEIGHT_3 = 900;

    public static final double ARM_POWER = 0.25;

    boolean bFirstIteration;


    @Override
    public void init() {

        // Get references to hardware.
        motorLeft = hardwareMap.get(DcMotor.class, "left_drive");
        motorRight = hardwareMap.get(DcMotor.class, "right_drive");
        motorArm = hardwareMap.get(DcMotor.class, "arm");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        rightClaw.setDirection(Servo.Direction.REVERSE);

        // Reset the encoders at ground level.
        motorArm.setMode(RunMode.STOP_AND_RESET_ENCODER);

        // We assume the arm is touching the ground at the beginning.
        armState = ArmState.STATE_H0;

        // Set direction of right motor to reverse.
        motorLeft.setDirection(Direction.REVERSE);

        // Set 0 power behavior.
        motorRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        gripperState = GripperState.STATE_STORAGE;

        dpadDownPressed = false;
        dpadUpPressed = false;

        bFirstIteration = true;

    }

    /**
     * This Op mode is a driver controlled Op Mode. It uses tank drive.
     */
    @Override
    public void loop() {

        if(bFirstIteration) {
            bFirstIteration = false;
            motorArm.setMode(RunMode.RUN_TO_POSITION);
            motorArm.setPower(ARM_POWER);
        }
        // get power from left and right joysticks.
        double leftPower = 0, rightPower = 0;
        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        // scale power if needed.

        // set motor power.
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        // set servo position.
        if(gamepad2.x) {
            gripperState = GripperState.STATE_OPEN;
        } else if (gamepad2.b) {
            gripperState = GripperState.STATE_CLOSED;
        }
        adjustGripper(gripperState);

        adjustArm();

        // send info back to Driver Station using telemetry.

        telemetry.addData("Power L / R", "%.02f / %.02f", motorLeft.getPower(), motorRight.getPower());
        telemetry.addData("Position L / R", "%d / %d",
                motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());
        telemetry.addData("arm state", armState);
        telemetry.addData("arm pos", motorArm.getCurrentPosition());

    }

    public void adjustGripper(GripperState currentState) {
        switch(currentState) {
            case STATE_STORAGE:
                leftClaw.setPosition(.1);
                rightClaw.setPosition(.1);
                break;
            case STATE_OPEN:
                leftClaw.setPosition(.75);
                rightClaw.setPosition(.75);
                break;
            case STATE_CLOSED:
                leftClaw.setPosition(.93);
                rightClaw.setPosition(.9);
                break;
            default:
                break;

        }
    }
    public void adjustArm(){

        if(gamepad2.dpad_up && dpadUpPressed == false){
            // user wants to raise arm.
            // Check current height, and if possible, raise the arm.
            if(armState == ArmState.STATE_H0){
                armState = ArmState.STATE_H1;
            }
            else if(armState == ArmState.STATE_H1){
                armState = ArmState.STATE_H2;
            }
            else if(armState == ArmState.STATE_H2){
                armState = ArmState.STATE_H3;
            }

            dpadUpPressed = true;

        } else if(gamepad2.dpad_up == false && dpadUpPressed == true){
            dpadUpPressed = false;
        }
        if(gamepad2.dpad_down && dpadDownPressed == false){
            // user wants to raise arm.
            // Check current height, and if possible, raise the arm.
            if(armState == ArmState.STATE_H3) {
                armState = ArmState.STATE_H2;
            } else if (armState == ArmState.STATE_H2) {
                armState = ArmState.STATE_H1;
            } else if (armState == ArmState.STATE_H1) {
                armState = ArmState.STATE_H0;
            }

            dpadDownPressed = true;

        } else if(gamepad2.dpad_down == false && dpadDownPressed == true){
            dpadDownPressed = false;
        }

        switch(armState){
            case STATE_H0:
                motorArm.setTargetPosition(HEIGHT_0);
                break;
            case STATE_H1:
                motorArm.setTargetPosition(HEIGHT_1);
                break;
            case STATE_H2:
                motorArm.setTargetPosition(HEIGHT_2);
                break;
            case STATE_H3:
                motorArm.setTargetPosition(HEIGHT_3);
                break;
            default:
                break;
        }

    }
}
