package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class RobotWebcam {
    private static final String VUFORIA_KEY =
            "AWkYdzb/////AAABmWsW4urs2EjOmK3aBEWJEFxEbGHh6an3+HFdzPIUoFs9oQJRm34hA528YQTZohbGdiZH5Gyot22SPfkQyhOgQMXlgfiQ0aTeNMSlD/FwWJ9kAjP1vIfFYnhEs9TjmpmPxKQD9OsxeA7hoKWgQWrOlNu0yG21oEWsr3FEh0rbAazVcNW27TFw5X1rK9mRdZT1/acPcntsbM74Sr/iy/nrp9y/rufNfwpzhnzkgFSJ6k5+6u6VEpE5Rg6M8kM6kfp+SCVNtqeYuiMV3RvKHoKuG4k+IOxd4uteb33+w8VNbIzoCMhuEP0iyb5hX+t5b6Tg6syLzkD5q9znjePjCxWx0MTFNtNe87Yvh2NBCYEwuOEo";
    VuforiaLocalizer locale;
    ClassFactory factory;

    public RobotWebcam() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        locale = factory.createVuforia(params);
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
