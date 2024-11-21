package org.firstinspires.ftc.teamcode.subSystems.old;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;


public class objectDetectionUtill {
    static {
        System.loadLibrary("task_vision_jni");
    }

    private static final String[] LABELS = {
            // I don't know why these labels work... they just do, just ignore them
            "right",
            "left",
            "none"
            //  "Red",
            //"Blue",
//            //"None"
//            "redObj",
    };
    private Telemetry telemetry;
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    //use during initialization
    public objectDetectionUtill(SampleMecanumDrive SampleHM, Telemetry telemetry){
        this.telemetry= telemetry;
        tfod = new TfodProcessor.Builder()
                .setModelAssetName("model.tflite")
                //   .setModelAssetName("RedandBlueModel2.tflite")
                //.setModelAssetName("model_unquant.tflite")
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(320)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(SampleHM.WebCamFront);
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.55f); //float
    }
    public void telemetryUpdate() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
        }   // end for loop
    }
}