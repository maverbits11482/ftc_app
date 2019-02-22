package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class AutonomousMavBotDrive extends LinearOpMode
{

    ThreeBot threeBot;
    public static final int ENCODER_DISTANCE = 1000;
    public static final double LIFT_POWER = .25;
    public static final double DRIVE_POWER = 1;
    ElapsedTime timer;
    boolean isRun = true;

    public void runOpMode()
    {

        threeBot = new ThreeBot(hardwareMap, this);
        // Reset Encoders and activate them, will run the drive until the robot is set down
        threeBot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Status:", "Ready");
        telemetry.update();

        this.waitForStart();

        while(!threeBot.isDescended() && opModeIsActive())
        {
            threeBot.raiseLifter(LIFT_POWER);
            telemetry.addData("Lifter:", "In Progress");
            telemetry.update();
        }

        threeBot.stopLifter();

        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorLeft.setTargetPosition(450);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorRight.setTargetPosition(450);

        threeBot.motorLeft.setPower(.6);
        threeBot.motorRight.setPower(.6);

        while(opModeIsActive() && threeBot.motorLeft.isBusy())
        {
            telemetry.addData("Status:", "unhooking");
        }

        threeBot.motorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorCenter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorCenter.setTargetPosition(threeBot.motorCenter.getCurrentPosition() + 1000);
        threeBot.motorCenter.setPower(.3);

        while(threeBot.motorCenter.isBusy() && opModeIsActive())
        {
            telemetry.addData("TIECenter Motor Pos", threeBot.motorCenter.getCurrentPosition());
            telemetry.addData("Center Target Pos", threeBot.motorCenter.getTargetPosition());
            telemetry.update();

        }
        threeBot.motorCenter.setPower(0);

//        while(threeBot.motorLeft.getCurrentPosition() <= ENCODER_DISTANCE && opModeIsActive())
//        {
//            threeBot.driveBot(DRIVE_POWER, DRIVE_POWER, 0);
//            telemetry.addData("Encoder Distance", threeBot.motorLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        threeBot.driveBot(0,0,0);
        telemetry.addData("Status:", "Finished");
        telemetry.update();

    }

}
