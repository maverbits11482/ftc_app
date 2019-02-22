package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "ThreeBot Autonomous:On Crater Side", group = "MavAutonomous")
public class HenryAutonomousTest extends LinearOpMode
{

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private int goldMineralLocation = 0;
    private double turningMotorPower = 0;
    private int width = 800;
    private boolean bFound;
    int goldMineralPos;
    BNO055IMU imu;

    final String VUFORIA_KEY = "AZqzp2L/////AAABmTYabB3IUER1vH8QW3FbJWYLgZjQuhIZP5ZAfYspmP7lu1xeiz6QuZLyIJY2trWKk4zGUwjUHBdUPAMWcbJYe4eYGHhxOOB9BshLvmc6i0G7xRkGTyVfNkfy/x4xvxJH8cCuhWRv9z7YyonDiBJQIJdpAl2g8jVKfC9jxt+2pNGpCavvr4syo1fF7nJlU0kM50Ohrkffp+djiSVV0P/cnBnt/ebAiWAHMs0QgiMBVQiA+8+0Wg8Pcy/8jJ/0kJ4mQAM5WJ3rqk1iam5gvyPM2zuGFahcJLNdClKY1/TTri3gsQc2Hr7LdXyxth7rX2jt3xBeRShr7xc4VsZv58H4dNNG7M6cxjm5FQQWyarjW8gB";

    ThreeBot threeBot;
    public static final double LIFT_POWER = .4;
    ElapsedTime timer;
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void runOpMode()
    {
        initVuforia();

        threeBot = new ThreeBot(hardwareMap, this);
        // Reset Encoders and activate them, will run the drive until the robot is set down
        threeBot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        threeBot.motorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Status:", "Ready");
        telemetry.update();

        // Wait for start button, put all instructions down below.
        this.waitForStart();

        while(!threeBot.isDescended() && opModeIsActive())
        {
            threeBot.raiseLifter(LIFT_POWER);
            telemetry.addData("Lifter:", "In Progress");
            telemetry.update();
        }

        threeBot.stopLifter();

        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorLeft.setTargetPosition(500);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorRight.setTargetPosition(500);

        threeBot.motorLeft.setPower(.4);
        threeBot.motorRight.setPower(.4);

        while(opModeIsActive() && threeBot.motorLeft.isBusy())
        {
            telemetry.addData("Status:", "unhooking");
            telemetry.update();
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
        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorLeft.setTargetPosition(-550);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorRight.setTargetPosition(-550);

        threeBot.motorLeft.setPower(.3);
        threeBot.motorRight.setPower(.3);

        while(opModeIsActive() && threeBot.motorLeft.isBusy())
        {
            telemetry.addData("Status:", "unhooking");
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

        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initIMU();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            // begin turning/searching.
            turnSearchLeft(0.1);

            while (opModeIsActive() && bFound == false) {
                // is Block visible?
                // if yes, stop turning and bFound = true;
                if(isGoldInView()) {
                    bFound = true;
                    RobotLog.vv("HCE", "Found Gold, exiting while loop");
                    threeBot.motorRight.setPower(0);
                    threeBot.motorLeft.setPower(0);
                }

                sleep(20);
                RobotLog.vv("LOOP", "Still in loop");
            }

            if (bFound) {
                telemetry.addData("Status", "found block");
            } else {
                telemetry.addData("Status", "Didn't find block");
            }
            telemetry.update();

        }


        driveForward(.3, 4100);
        while(threeBot.motorLeft.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to position");
            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }



        telemetry.update();


    }

    public void turnSearchLeft(double power) {
        threeBot.motorLeft.setPower(-power);
        threeBot.motorRight.setPower(power);
    }

    public boolean isGoldInView() {
        boolean bVal=false;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            Orientation Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double headerAngle = AngleUnit.DEGREES.normalize(Angle.firstAngle);
            if (updatedRecognitions != null)
            {
                if (updatedRecognitions.size() > 0)
                {
                    for (Recognition recognition : updatedRecognitions)
                    {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                        {
                            if(recognition.getLeft() > 350)
                                bVal = true;
                        }
                    }
                }
            }
        }

        return bVal;
    }
    public void driveForward(double power, int encoderDistance)
    {
        RobotLog.vv("HCE", "Entered driveForwardMethod");
        threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorLeft.setTargetPosition(threeBot.motorLeft.getCurrentPosition() + encoderDistance);

        threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        threeBot.motorRight.setTargetPosition(threeBot.motorRight.getCurrentPosition() + encoderDistance);

        threeBot.motorLeft.setPower(power);
        threeBot.motorRight.setPower(power);
    }
    public void resetIMU()
    {

    }
    public void getAngle()
    {

    }

    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

}
