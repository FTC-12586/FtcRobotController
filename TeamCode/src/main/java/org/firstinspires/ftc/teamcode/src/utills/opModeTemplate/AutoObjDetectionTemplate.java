package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.src.utills.VuforiaKey;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A template for all Autonomous opModes that use Vision, allows easy initialization
 */
@Disabled
public abstract class AutoObjDetectionTemplate extends AutonomousTemplate {

    /**
     * The file that is loaded to detect from
     */
    private static final String TFOD_MODEL_ASSET = "Trained Pink Team Marker Finder Mk2.tflite";

    /**
     * The labels in the file
     */
    private static final String[] LABELS = {"Pink Team Marker"};

    /**
     * A object to lock on to for the thread safety, used in _initVuforia
     */
    private static final Lock VF_Lock = new ReentrantLock();

    /**
     * A object to lock on to for the thread safety, used in _initTfod
     */
    private static final Lock TFOD_Lock = new ReentrantLock();

    /**
     * vuforia object
     */
    public volatile VuforiaLocalizer vuforia;

    /**
     * Tensorflow Object Detection Object
     */
    public volatile TFObjectDetector tfod;

    /**
     * Initializes the Vuforia object
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void initVuforia() throws InterruptedException {
        telemetry.addData("Vuforia initialization:", "Started");
        telemetry.update();

        checkStop();
        this._initVuforia();
        checkStop();

        telemetry.addData("Vuforia initialization:", "Complete");
        telemetry.update();

    }

    /**
     * Does the initialization, provides synchronization for thread safety
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    private void _initVuforia() throws InterruptedException {
        //does the initialization
        VuforiaLocalizer Vuforia;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VuforiaKey.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, GenericOpModeTemplate.LeftWebcamName);

        //Waits for mutex to be available

        checkStop();
        VF_Lock.lockInterruptibly();
        try {
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        } finally {
            VF_Lock.unlock();
        }
        checkStop();


    }

    /**
     * Initializes the Tensorflow Object Detection Object object
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void initTfod() throws InterruptedException {
        telemetry.addData("TFOD initialization:", "Started");
        telemetry.update();
        checkStop();

        this._initTfod();

        checkStop();
        telemetry.addData("TFOD initialization:", "Complete");
        telemetry.update();

    }

    /**
     * Does the initialization, provides synchronization for thread safety
     */
    private void _initTfod() throws InterruptedException {

        //Some pre-initialization
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        //Waits for mutex to be available
        TFOD_Lock.lockInterruptibly();
        try {
            //Runs initialization Code
            this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        } finally {
            TFOD_Lock.unlock();
        }

    }

    /**
     * Activates Tensorflow
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void activateTF() throws InterruptedException {
        checkStop();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.4, 16.0 / 9.0);
        }
        checkStop();
    }


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll() throws InterruptedException {
        initVuforia();
        initTfod();
        activateTF();
        super.initAll();
    }

    /**
     * It uses the camera to determine where the object is on the screen
     *
     * @param arraySize The number of samples to take
     * @param sleepTime How long to wait between each sample
     * @return It returns the average position of all the samples
     * @throws InterruptedException Throws exception if the opMode is stopped during function execution
     */
    public BarcodePositions getAverageOfMarker(int arraySize, int sleepTime) throws InterruptedException {

        BarcodePositions[] markerPositions = new BarcodePositions[arraySize];

        for (int i = 0; i < arraySize; i++) {
            markerPositions[i] = this.findPositionOfMarker();
            Thread.sleep(sleepTime);
            checkStop();
        }

        int sum = 0;
        for (int i = 0; i < arraySize; i++) {
            switch (markerPositions[i]) {
                case NotSeen:
                    break;
                case Right:
                    sum++;
                    break;
                case Left:
                    sum = sum + 2;
                    break;
                case Center:
                    sum = sum + 3;
                    break;
            }
        }

        int result = (int) Math.round(sum / (double) arraySize);


        switch (result) {
            case 1:
                return BarcodePositions.Right;
            case 2:
                return BarcodePositions.Left;
            case 3:
                return BarcodePositions.Center;
            default:
                return BarcodePositions.NotSeen;
        }
    }

    /**
     * Uses the camera to determine where the object is on screen
     *
     * @return Where the marker is
     */
    public BarcodePositions findPositionOfMarker() {
        final int screenWidth = 1260;

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
            Recognition rec = updatedRecognitions.get(0);
            double xCenterLine = ((rec.getRight() + rec.getLeft()) / 2.0);

            if (xCenterLine < 450) {
                return BarcodePositions.Left;
            } else if (xCenterLine < 820) {
                return BarcodePositions.Center;
            } else {
                return BarcodePositions.Right;
            }
        } else {
            return BarcodePositions.NotSeen;
        }
    }

}

