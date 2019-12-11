package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class RobotWebcam {
    private static final String VUFORIA_KEY =
            "ARuYIcn/////AAABmQk2dFO/OUiulDemwSzvz5IhKlYZ2CmzXW7BPOWkTFPL9s3RAjUawH6QIgjWexqwmjcJsc4i0OwJLUjjpS2u9AWLZHnehVawjKWrPmar8c9gF8OGRWBDod9yM1jhUe2aSf1a31KPrUNpATyt+3dBgxWba3nY8Yhpqt8xiUjucSkHOY5iJ9KEOSt1OMGXmX65FPsptoVSnZWq+kRW1QQ9NBGi6+uEeODyy27XLeDDOw/Y5QuRv3AUqCsSzgc0af+SE0OL/ZRJhDt+5Xal1Y7UCzbF7paGA8RjnY8+ZfJQx1xcbf1lJZ5nmaQ4EaOBwrUUYxF20XL3gpbO4Z5B6yNml1olDk4myGif9NrW2KOWllx4";
    VuforiaLocalizer locale;
    int cameraMonitorViewId;

    public RobotWebcam(HardwareMap hwMap) {
        this.cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        locale = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        locale.enableConvertFrameToBitmap();
    }

    public Frame getFrame() throws InterruptedException {
        return locale.getFrameQueue().take();
    }

    public Image getImage() throws InterruptedException {
        /*To access the image: you need to iterate through the images of the frame object:*/

        Frame frame = this.getFrame();
        Image rgb = null;

        long numImages = frame.getNumImages();


        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        return rgb;
    }

    public Bitmap getBitmap() throws InterruptedException {
        Frame frame = this.getFrame();
        return locale.convertFrameToBitmap(frame);
    }
}
