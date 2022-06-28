package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.ouropencv.ContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * A template for all Autonomous opModes that use OpenCV Vision, allows easy initialization
 */
@SuppressWarnings("unused")
@Disabled
public abstract class AutoObjDetectionTemplateCV extends AutonomousTemplate {

    private static final double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    private static final double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    private static final double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    private static final double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    // Pink Range                                      Y      Cr     Cb
    private static final Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    private static final Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private static final int CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE = -1;
    private static final int CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE = -2;
    private final static ContourPipeline myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY);

    static {
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
    }


    private OpenCvWebcam webcam;

    /**
     * Constructs an anonymous listener
     */
    private final OpenCvCamera.AsyncCameraOpenListener defaultListener = new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            try {
                //Trying to open camera at given resolution
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            } catch (final OpenCvCameraException e) {
                /*
                Fallback code if the camera fails to open.
                Sometimes the camera will fail to open even though the camera actually supports the given resolution
                However, the error message suggests alternative resolutions that will work
                 */

                if (isRecoverable(e)) {
                    //If we are here, we can possibly recover
                    assert (e.getMessage() != null);
                    final int[] newRes = getLargestResolution(getResolutionsFromException(e));
                    webcam.startStreaming(newRes[0], newRes[1], OpenCvCameraRotation.UPRIGHT);
                    RobotLog.addGlobalWarningMessage("Camera failed to open because of resolution error, but recovered");
                } else {
                    //The message returned no resolutions we can use, throw the exception up the stack
                    throw e;
                }
            }
        }

        @Override
        public void onError(int errorCode) {

        }
    };

    /**
     * Parses out possible resolutions from a valid {@link OpenCvCameraException}
     *
     * @param exception a valid {@link OpenCvCameraException}
     * @return An array of possible resolutions
     */
    @SuppressLint("NewApi")
    private static int[][] getResolutionsFromException(final OpenCvCameraException exception) {

        final String errorMessage = exception.getMessage();

        assert (errorMessage != null);

        //Trim off fluff
        String strSizes = errorMessage.substring(errorMessage.indexOf('['), errorMessage.lastIndexOf(']') + 1);

        //Count number of valid resolutions passed back
        final long count = strSizes.chars().filter(ch -> ch == ',').count() + 1;

        final int[][] returnContainer = new int[(int) count][2];

        strSizes = strSizes.replaceAll("[,]", " ");

        int storageIndex = 0;

        while (strSizes.length() > 0) {
            //Gets the resolution without brackets, eg: 176x144
            final String resolution = strSizes.substring(strSizes.indexOf('[') + 1, strSizes.indexOf(']'));
            final String[] xy = resolution.split("[x]");
            returnContainer[storageIndex][0] = Integer.parseInt(xy[0]);
            returnContainer[storageIndex][1] = Integer.parseInt(xy[1]);

            storageIndex++;


            strSizes = strSizes.substring(strSizes.indexOf(']') + 1);


        }


        return returnContainer;
    }

    /**
     * Returns the largest resolution in terms of area from a array of possible resolutions
     *
     * @param resolutions An array of possible resolutions
     * @return The largest resolution in terms of area from a array of possible resolutions
     */
    private static int[] getLargestResolution(final int[][] resolutions) {
        long maxSize = 0;
        int maxIndex = -1;
        for (int index = 0; index < resolutions.length; index++) {
            int[] x = resolutions[index];

            long size = (long) x[0] * x[1];
            if (size > maxSize) {
                maxSize = size;
                maxIndex = index;
            }
        }
        assert maxIndex >= 0;

        return resolutions[maxIndex];

    }

    /**
     * Checks a message from an {@link OpenCvCameraException} to see if it has suggested resolutions we can use
     *
     * @param e An OpenCvCameraException object
     * @return true if it contains acceptable resolutions, false if no resolutions are found
     */
    private static boolean isRecoverable(OpenCvCameraException e) {

        final String message = e.getMessage();

        //null check
        if (message == null) {
            return false;
        }

        //Asserts that the message has brackets which indicate the resolution
        if (message.contains("[") && message.contains("]")) {
            return message.indexOf('[') < message.lastIndexOf(']');
        }
        return false;
    }

    /**
     * Initializes all fields provided by this class
     *
     * @param DefaultCameraName The name the OpMode should initialize with by default
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll(String DefaultCameraName) throws InterruptedException {
        this.initOpenCV(DefaultCameraName);
        super.initAll();
    }

    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll() throws InterruptedException {
        checkStop();
        this.initOpenCV(GenericOpModeTemplate.RightWebcamName);
        checkStop();
        super.initAll();
        checkStop();
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
     * @return Where the marker is or not seen if camera is not initialized
     */
    public BarcodePositions findPositionOfMarker() {
        if (webcam != null) {
            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 400) {
                    return BarcodePositions.Right;
                } else if (myPipeline.getRectMidpointX() > 200) {
                    return BarcodePositions.Center;
                } else {
                    return BarcodePositions.Left;
                }
            }
        }
        return BarcodePositions.NotSeen;
    }

    /**
     * Initializes OpenCV on a camera
     *
     * @param CameraName The name of the camera to start OpenCV on
     * @throws InterruptedException Throws if the opMode is stopped during initialization
     */
    public void initOpenCV(String CameraName) throws InterruptedException {
        RobotLog.d("OpenCV Init Started");

        checkStop();

        WebcamName camName = hardwareMap.get(WebcamName.class, CameraName);

        checkStop();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        checkStop();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);

        checkStop();

        webcam.setPipeline(myPipeline);

        // Webcam Streaming
        checkStop();

        webcam.openCameraDeviceAsync(defaultListener);
        RobotLog.d("OpenCV Init Finished");


    }

    /**
     * Closes the webcam and logs it in the Logcat
     */
    protected void closeWebcam() {
        if (webcam != null) {
            webcam.closeCameraDeviceAsync(() -> RobotLog.dd("Webcam Image Processor", "Closed"));
            webcam = null;
        }
    }

    /**
     * Cleans up the OpMode
     */
    @Override
    protected void cleanup() {
        super.cleanup();
        closeWebcam();
    }

}


