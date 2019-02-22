package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Henry on 11/11/2017.
 */

@TeleOp
public class ThreeBotDrive extends OpMode
{

    enum LIFTER_STATE {MANUAL_MODE, AUTO_LIFT, AUTO_DESCEND}

    ThreeBot threeBot;

    double left;
    double right;
    double drive;
    double turn;
    double max;
    boolean wantsRaise = false;

    double liftPower = 1;

    double sideways;

    LIFTER_STATE lifterState;

    @Override
    public void init()
    {
        threeBot = new ThreeBot(hardwareMap, this);

        lifterState = LIFTER_STATE.MANUAL_MODE;

        // Prompt user to push start button
        threeBot.activateShovel(telemetry);
        telemetry.addData(">>","Press start to begin");
    }

    private void adjustLifter() {
        if(gamepad2.y && !threeBot.isLifted())
        {
            // put it in auto descend mode.
            lifterState = LIFTER_STATE.AUTO_LIFT;
        }
        else if(gamepad2.dpad_up)
        {
            lifterState = LIFTER_STATE.MANUAL_MODE;
            threeBot.raiseLifter(liftPower);
        } else if (gamepad2.dpad_down)
        {
            lifterState = LIFTER_STATE.MANUAL_MODE;
            threeBot.lowerLifter(liftPower);
        } else if (lifterState == LIFTER_STATE.MANUAL_MODE)
        {
            threeBot.stopLifter();
        }

        // do we need to autolift the arm?
        if (lifterState == LIFTER_STATE.AUTO_LIFT) {
            if(threeBot.isLifted()) {
                threeBot.stopLifter();
            } else {
                // we want to lower the lifter which raises the robot.
                threeBot.raiseLifter(liftPower);
            }
        } else if (lifterState == LIFTER_STATE.AUTO_DESCEND) {
            if (threeBot.isDescended()) {
                threeBot.stopLifter();
            } else {
                // we want to raise the lifter which lowers the robot (descends).
                threeBot.raiseLifter(liftPower);
            }
        }
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

        // adjust lifter
        adjustLifter();

        telemetry.addData("Red Hue", threeBot.lightSensor.red());
        telemetry.addData("isLifted", threeBot.isLifted());


        if(gamepad1.left_trigger > .5)
            threeBot.harvesterIn(.7);
        else if(gamepad1.right_trigger > .5)
            threeBot.harvesterOut(.7);
        else
            threeBot.motorShovel.setPower(0);
            telemetry.addData("Harvester Status:", "Not active");


        telemetry.addData("left", "%.02f", threeBot.motorLeft.getPower());
        telemetry.addData("right", "%.02f", threeBot.motorLeft.getPower());
        telemetry.addData("center", "%.02f", threeBot.motorLeft.getPower());
        telemetry.addData("distance from ground", threeBot.distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
