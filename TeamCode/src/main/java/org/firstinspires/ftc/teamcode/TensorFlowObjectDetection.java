package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.jar.Attributes;

@Autonomous
@Disabled
public class TensorFlowObjectDetection extends OpMode
{
    final int LEFT_BOUND = 350;
    final int RIGHT_BOUND = 450;
    private int goldMineralLocation;
    private int correction;
    private boolean isFound;
    private boolean isActive;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    final String VUFORIA_KEY = "AZqzp2L/////AAABmTYabB3IUER1vH8QW3FbJWYLgZjQuhIZP5ZAfYspmP7lu1xeiz6QuZLyIJY2trWKk4zGUwjUHBdUPAMWcbJYe4eYGHhxOOB9BshLvmc6i0G7xRkGTyVfNkfy/x4xvxJH8cCuhWRv9z7YyonDiBJQIJdpAl2g8jVKfC9jxt+2pNGpCavvr4syo1fF7nJlU0kM50Ohrkffp+djiSVV0P/cnBnt/ebAiWAHMs0QgiMBVQiA+8+0Wg8Pcy/8jJ/0kJ4mQAM5WJ3rqk1iam5gvyPM2zuGFahcJLNdClKY1/TTri3gsQc2Hr7LdXyxth7rX2jt3xBeRShr7xc4VsZv58H4dNNG7M6cxjm5FQQWyarjW8gB";

   // ThreeBot threeBot;

    VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    enum RobotState
    {
        STATE_INIT,
        STATE_MOVE_LEFT,
        STATE_MOVE_RIGHT,
        STATE_STOP,
        STATE_PROPORTIONAL_CONTROL;
    }
    RobotState currRobotState = RobotState.STATE_INIT;

    public void init()
    {
       // threeBot = new ThreeBot(hardwareMap, this);

        initVuforia();

        initTfod();
        isActive = false;

        if (tfod != null) {
            tfod.activate();
            isActive = true;
        }
    }
    public void loop()
    {


        isFound = false;

//      switch(currRobotState)
//        {
//            case STATE_INIT: initState();
//                break;
//            case STATE_STOP:
//
//
//        }
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("size", updatedRecognitions.size());

//                for (Recognition recognition : updatedRecognitions)
//                {
//                    telemetry.addData("Recognition label", recognition.getLabel());
//                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
//                    {
//                        goldMineralLocation = (int) recognition.getLeft();
//                        isFound = true;
//                    }
//                }
            } else {
                telemetry.addData("size", "NULL");
            }

        }




//
//        }

//        if(isFound)
//        {
//            telemetry.addData("Status:", "Gold Mineral Found");
//            telemetry.addData("Gold Mineral Position", goldMineralLocation);
//        }
//        else
//        {
//            telemetry.addData("Gold Mineral Position:", "Cannot Be located");
//        }

        //telemetry.update();

    }
    @Override
    public void stop()
    {
        if (tfod != null) {
            tfod.shutdown();
        }

    }
    private void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
       // parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
    private void initState()
    {

    }

}
