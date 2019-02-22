package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
@Autonomous(name = "Tensor Flow autonomous drive")
public class TensorFlowAutonomousTracking extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    ThreeBot threeBot;

    boolean isCentered = false;
    int goldMineralCenter = 0;

    @Override
    public void runOpMode()
    {
        //Initialize vuforia detection software.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AYzsBFX/////AAABmYWZ9Nn6nUO0iulJMi7KfpcARkVJgR169Y4MHI/FNfFhMdaCEu6qKETkKWelCINLyC1+V1Yz4+s5c4J1gK3wD32nBL9xEmxn1rX6/4iAWqIH6NkVC6BonShN6yOlMFjcT5KAYIQjus+7u7VWbYmL7d77kmOlxpFBYlRAj2H5kYAFkberxLEjDnXM5uPfDi9hOWgLwIZa4HFZKwFAQ+kF6H3o+zaWh3Ob5+1MgcVOjiR5WrKW1DHTP/S4LSla6e1QOBiQyVvmy57zoNvZmDpRvRppk90NjM9yEmm8NPAgqR8m2zJ4g9DVx5PTyW0Ue2htBKiooQnF7fgMRTSe7fQuqRCUwqIt9VrBJxkYmcVDlrie";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Initialize TensorFlow Object Detection
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        threeBot = new ThreeBot(hardwareMap, this);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while(opModeIsActive() && !isCentered) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());



                for (Recognition recognition : updatedRecognitions)
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                        goldMineralCenter = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
                if(goldMineralCenter >= 620 && goldMineralCenter <= 660)
                    isCentered = true;
                else
                {
                    threeBot.driveBot(0,0, (620 - goldMineralCenter) * .1);
                }
                telemetry.addData("Center Position of Gold: ", goldMineralCenter);
//                telemetry.addData("Power: ", (620 - goldMineralCenter) * .1));
                telemetry.update();
            }
            else
            {
                threeBot.driveBot(0,0,0);
            }

        }
    }
}
