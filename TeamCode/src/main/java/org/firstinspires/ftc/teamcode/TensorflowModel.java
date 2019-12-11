//package org.firstinspires.ftc.teamcode;
//
//import android.annotation.TargetApi;
//import android.graphics.Bitmap;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.tensorflow.lite.DataType;
//import org.tensorflow.lite.Interpreter;
//import org.tensorflow.lite.support.image.ImageProcessor;
//import org.tensorflow.lite.support.image.TensorImage;
//import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;
//
//import java.io.File;
//import java.io.IOException;
//import java.nio.ByteBuffer;
//import java.nio.FloatBuffer;
//import java.nio.file.Files;
//
//@TargetApi(26)
//public class TensorflowModel {
//    RobotWebcam webcam;
//    String TFOD_MODEL_ASSET = "small_model-2.tflite";
//    Interpreter interpreter;
//    ByteBuffer tflite_model;
//    float[] data = {0, 0, 0};
//    FloatBuffer outputdata;
//
//    TensorBuffer output;
//    int[] shape = {3};
//    DataType outputDatatype = DataType.FLOAT32;
//
//
//    public TensorflowModel(HardwareMap hwmap) {
//
//        //Tensor output = output.create();
//        webcam = new RobotWebcam(hwmap);
//    }
//
//    void initTFLite() throws IOException {
//        File file = new File(TFOD_MODEL_ASSET);
//        byte[] fileContent = Files.readAllBytes(file.toPath());
//        tflite_model = ByteBuffer.wrap(fileContent);
//        /*FileInputStream f_input_stream= new FileInputStream(new File(TFOD_MODEL_ASSET));
//        FileChannel f_channel = f_input_stream.getChannel();
//        tflite_model = f_channel.map(FileChannel.MapMode.READ_ONLY, 0, f_channel .size());*/
//
//        interpreter = new Interpreter(tflite_model);
//    }
//
//    public float[] predictPos() throws InterruptedException {
//        try {
//            initTFLite();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//        ByteBuffer vbb = ByteBuffer.allocateDirect(12);
//        outputdata = vbb.asFloatBuffer();
//        outputdata.put(data);
//
//        // Creates the output tensor and its processor.
//        /*output = TensorBuffer.createFixedSize(shape, outputDatatype);
//        output.loadArray(outputdata);*/
//
//
//        Bitmap bitmap = webcam.getBitmap();
//        bitmap = resizeBitmap(bitmap, 100, 75);
//        // Loads bitmap into a TensorImage.
//        TensorImage imageBuffer = new TensorImage(outputDatatype);
//        imageBuffer.load(bitmap);
//
//        // Creates processor for the TensorImage.
//        ImageProcessor imageProcessor =
//                new ImageProcessor.Builder()
//                        .build();
//        imageProcessor.process(imageBuffer);
//        interpreter.run(imageBuffer, outputdata);
//        interpreter.close();
//        return output.getFloatArray();
//    }
//
//    public void closeInterpreter() {
//        interpreter.close();
//    }
//
//    private Bitmap ARGBBitmap(Bitmap img) {
//        return img.copy(Bitmap.Config.ARGB_8888,true);
//    }
//
//    private static Bitmap resizeBitmap(Bitmap bf, int width, int height) {
//        bf.reconfigure(width, height, Bitmap.Config.ARGB_8888);
//        return bf;
//    }
//
//}
