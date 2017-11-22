package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Henry on 11/21/2017.
 */
@TeleOp
public class ServoTest extends LinearOpMode {
    double servoPos = .5;
    
    public static final double servoChange = .0025;

    Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.servo.get("servo");

        servo.setPosition(servoPos);

        telemetry.addData(">>", "Press Play to start");

        telemetry.update();

        this.waitForStart();

        while(opModeIsActive())
        {

            if(gamepad1.dpad_right && servoPos < 1)
            {
                servoPos = servoPos + servoChange;
            }
            else if(gamepad1.dpad_left && servoPos > 0)
            {
                servoPos = servoPos - servoChange;
            }

            telemetry.addData("Servo Position", servoPos);
            telemetry.update();

            servo.setPosition(servoPos);

        }

    }
}
