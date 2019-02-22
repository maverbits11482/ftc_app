package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Encodertest extends LinearOpMode
{
    public void runOpMode()
    {
        ThreeBot threeBot = new ThreeBot(hardwareMap, this);

        threeBot.motorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.waitForStart();

        threeBot.motorCenter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorCenter.setTargetPosition(1000);
        threeBot.motorCenter.setPower(.5);

        while(opModeIsActive())
        {
            telemetry.addData("Right Motor:", threeBot.motorRight.getCurrentPosition());
            telemetry.addData("Left Motor:", threeBot.motorLeft.getCurrentPosition());
            telemetry.addData("Center Motor:", threeBot.motorCenter.getCurrentPosition());
            telemetry.addData("Center Motor Target:", threeBot.motorCenter.getTargetPosition());
            telemetry.addData("Center Motor Power:", threeBot.motorCenter.getPower());
            telemetry.update();
        }

    }
}
