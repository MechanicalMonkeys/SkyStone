package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.tensorflow.lite.Interpreter;

import java.io.File;

public class TensorflowModel {
    RobotWebcam webcam;
    TFObjectDetector tfod;
    TFObjectDetector.Parameters tfodparam;
    VuforiaLocalizer.Parameters vuforiaparam;
    String TFOD_MODEL_ASSET = "small_model.tflite";
    File modelFile = new File(TFOD_MODEL_ASSET);
    VuforiaLocalizer vuforia;
    Interpreter interpreter;
    float[] output = new float[3];

    public TensorflowModel() {
        initVuforia();
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodparam, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET);
    }

    void initTFLite() {
        Interpreter interpreter = new Interpreter(modelFile);
    }

    private void initVuforia() {
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaparam);
    }

    public float[] predictPos() throws InterruptedException {
        interpreter.run(webcam.getBitmap(), output);
        return output;
    }

    public void closeInterpreter() {
        interpreter.close();
    }

}
