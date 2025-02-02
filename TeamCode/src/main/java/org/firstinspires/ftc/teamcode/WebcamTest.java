package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class WebcamTest {
    /* Constants */
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final double MIN_CONFIDENCE = 0.1;
    private static final int NUM_BLOCKS = 3;
    int cameraMonitorViewId;

    /* Arrays for skystone position detection */
    private static float[] left_vals = {0, 0, 0};
    private static String[] label_vals = {"", "", ""};

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ARuYIcn/////AAABmQk2dFO/OUiulDemwSzvz5IhKlYZ2CmzXW7BPOWkTFPL9s3RAjUawH6QIgjWexqwmjcJsc4i0OwJLUjjpS2u9AWLZHnehVawjKWrPmar8c9gF8OGRWBDod9yM1jhUe2aSf1a31KPrUNpATyt+3dBgxWba3nY8Yhpqt8xiUjucSkHOY5iJ9KEOSt1OMGXmX65FPsptoVSnZWq+kRW1QQ9NBGi6+uEeODyy27XLeDDOw/Y5QuRv3AUqCsSzgc0af+SE0OL/ZRJhDt+5Xal1Y7UCzbF7paGA8RjnY8+ZfJQx1xcbf1lJZ5nmaQ4EaOBwrUUYxF20XL3gpbO4Z5B6yNml1olDk4myGif9NrW2KOWllx4";

    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    private VuforiaLocalizer vuforia;

    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //    public void runOpMode() {
//        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//        // first.
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        /*
//         * Activate TensorFlow Object Detection before we wait for the start command.
//         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
//         */
//        if (tfod != null) {
//            tfod.activate();
//        }
//
//        /* Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        if (updatedRecognitions.size() == NUM_BLOCKS) { // Camera detected a certain number of blocks
//                            //telemetry.addData("# Object Detected", updatedRecognitions.size());
//                            // step through the list of recognitions and display boundary info.
//                            int i = 0;
//                            for (Recognition recognition : updatedRecognitions) {
//                                /*
//                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                        recognition.getLeft(), recognition.getTop());
//                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                        recognition.getRight(), recognition.getBottom());
//                                        */
//                                label_vals[i] = recognition.getLabel();
//                                left_vals[i] = recognition.getLeft();
//                                i++; // Bug Fix - Counter Increment
//                            }
//                            int position;
//                            position = getSkystonePosition(label_vals, left_vals);
//                            switch (position) {
//                                case -1:
//                                    // Display the results
//                                    telemetry.addData("Skystone Position", "LEFT");
//                                    telemetry.update();
//                                    break;
//                                case 0:
//                                    // Display the results
//                                    telemetry.addData("Skystone Position", "CENTER");
//                                    telemetry.update();
//                                    break;
//                                case 1:
//                                    // Display the results
//                                    telemetry.addData("Skystone Position", "RIGHT");
//                                    telemetry.update();
//                                    break;
//                            }
//                        }
//                    }
//                }
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
//    }
//*/
    public void init(HardwareMap ahwmap) {
        HardwareMap hwMap = ahwmap;
        this.cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(hwMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hwMap);
        }
    }

    private void InsertionSort(float[] arr)
    {
        int n = arr.length;
        for (int i = 1; i < n; ++i) {
            float key = arr[i];
            int j = i - 1;

            /* Move elements of arr[0..i-1], that are
               greater than key, to one position ahead
               of their current position */
            while (j >= 0 && arr[j] > key) {
                arr[j + 1] = arr[j];
                j = j - 1;
            }
            arr[j + 1] = key;
        }
    }

    public int getSkystonePosition(String[] labels, float[] vals) { // 1=right, 0=center, -1=left
        float skystoneVal = 0;
        int pos;

        for (int i = 0; i < 3; i++) {
            String object = labels[i];
            if (object.equals(LABEL_SECOND_ELEMENT)) {
                skystoneVal = vals[i]; // Get the left position of the skystone
                break; // Exit the loop
            }
        }

        InsertionSort(vals); // Sort the array - smallest values in front
        float val1 = vals[0];
        float val2 = vals[1];
        float val3 = vals[2];
        /* Find the placement of the skystone once values have been sorted */
        if (skystoneVal == val1) { // Left side
            pos = 0;
        } else if (skystoneVal == val2) { // Center
            pos = 1;
        } else if (skystoneVal == val3) { // Right
            pos = 2;
        } else {
            pos = 3;
        }
        return pos;
    }

    /*
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap ahwmap) {
        HardwareMap hwMap = ahwmap;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /*
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap ahwmap) {
        int tfodMonitorViewId = ahwmap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwmap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public int detectSkystonePosition(OpMode opmode) {

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == NUM_BLOCKS) { // Camera detected a certain number of blocks
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {

                        opmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        opmode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        opmode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        label_vals[i] = recognition.getLabel();
                        left_vals[i] = recognition.getLeft();
                        i++; // Bug Fix - Counter Increment
                    }
                    int position;
                    position = getSkystonePosition(label_vals, left_vals);
                    //tfod.shutdown();
                    return position;
                }
            }
        }
        return 3; // if something goes wrong
    }
}



