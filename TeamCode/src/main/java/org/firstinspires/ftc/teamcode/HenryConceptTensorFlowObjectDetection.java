package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Henry Concept: TensorFlow Object Detection", group = "Concept")
public class HenryConceptTensorFlowObjectDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private int goldMineralLocation = 0;
    private int height;
    private int width;
    private double centerMotorPower;


    final String VUFORIA_KEY = "AZqzp2L/////AAABmTYabB3IUER1vH8QW3FbJWYLgZjQuhIZP5ZAfYspmP7lu1xeiz6QuZLyIJY2trWKk4zGUwjUHBdUPAMWcbJYe4eYGHhxOOB9BshLvmc6i0G7xRkGTyVfNkfy/x4xvxJH8cCuhWRv9z7YyonDiBJQIJdpAl2g8jVKfC9jxt+2pNGpCavvr4syo1fF7nJlU0kM50Ohrkffp+djiSVV0P/cnBnt/ebAiWAHMs0QgiMBVQiA+8+0Wg8Pcy/8jJ/0kJ4mQAM5WJ3rqk1iam5gvyPM2zuGFahcJLNdClKY1/TTri3gsQc2Hr7LdXyxth7rX2jt3xBeRShr7xc4VsZv58H4dNNG7M6cxjm5FQQWyarjW8gB";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    ThreeBot threeBot;

    @Override
    public void runOpMode() {

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
        threeBot = new ThreeBot(hardwareMap, this);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            /*
            while(!threeBot.isDescended() && opModeIsActive())
            {
                threeBot.raiseLifter(.3);
                telemetry.addData("Lifter:", "In Progress");
                telemetry.update();
            }
            threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            threeBot.motorLeft.setTargetPosition(450);
            threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            threeBot.motorRight.setTargetPosition(450);

            threeBot.motorLeft.setPower(.6);
            threeBot.motorRight.setPower(.6);
            while(opModeIsActive() && threeBot.motorLeft.isBusy() && threeBot.motorLeft.isBusy())
            {
                telemetry.addData("Status:", "Unhooking");
            }

            threeBot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            threeBot.motorRight.setTargetPosition(3000);
            threeBot.motorRight.setPower(.5);

            while(threeBot.motorRight.isBusy())
            {
                telemetry.addData("Status:", "turning");
            }
            threeBot.motorRight.setPower(0);
            threeBot.motorLeft.setPower(0);
            */
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null)
                    {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      for(Recognition recognition: updatedRecognitions)
                      {
                          telemetry.addData("Recognition label", recognition.getLabel());
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                          {
                              width = recognition.getImageWidth();
                              goldMineralLocation = (int) (recognition.getLeft() + recognition.getRight()) / 2;
                              telemetry.addData("Gold Mineral Location", goldMineralLocation);
                              telemetry.addData("Image Width", width);
                          }
                      }
                      int rightLength = (width / 2) + 50;
                        int leftLength = (width / 2) - 50;
                      if(goldMineralLocation >= leftLength && goldMineralLocation <= rightLength)
                      {
                          centerMotorPower = 0;
                      }
                      else if(goldMineralLocation < leftLength)
                      {
                          centerMotorPower = .4;
                      }
                      else if(goldMineralLocation > rightLength)
                      {
                          centerMotorPower = -.4;
                      }

                      if(centerMotorPower == 0) {
                          threeBot.driveBot(0, 0, centerMotorPower);

                          threeBot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                          threeBot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                          threeBot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                          threeBot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                          threeBot.motorLeft.setTargetPosition(3000);
                          threeBot.motorRight.setTargetPosition(3000);
                          threeBot.motorLeft.setPower(.3);
                          threeBot.motorRight.setPower(.3);
                      }

                      while(threeBot.motorLeft.isBusy() && opModeIsActive())
                      {
                          telemetry.addData("Status:", "Driving towards block");
                      }
                      telemetry.addData("MotorPower", centerMotorPower);
                      telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

//    private void initVuforia() {
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CameraDirection.BACK;
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//    }
    private void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia = ClassFactory.getInstance().createVuforia(parameters);



        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
