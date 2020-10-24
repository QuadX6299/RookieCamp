package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;


public class Vision {

    private LinearOpMode opMode;

    private VuforiaLocalizer vuforia;

    private final int RED_THRESHOLD = 200;
    private final int GREEN_THRESHOLD = 100;
    private final int BLUE_THRESHOLD = 50;

    public Vision(LinearOpMode opMode) {
        this.opMode = opMode;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVAI/2z/////AAABmTc9AU8cVUOfl7vuh8ikiF0e7FU3OFAEPoxa/VlSyx0onR7NwQ33Dsgu1WW083pvmbyREqVjl4BEHOsuBXj5cYF/JEHrwvY293Km9pkBHi5rxwMF1XW94UVqIlMF7JT47DjM3qwiOPOY+FXvdv4YsD+jmP7ravsSI0Vo1X0i4KhnHkfi89UhMRoE4fN3y+oAbAzS1l7ze/Ca+h2BSD2Co3hb96j47qhv4KgOTnZ9GOZS3rz/Rt9llF7C/dKRXR+aYdfFketG8VXSifBEJA6JiYQX3Jj2A8plqvCiokrfWX61Y2+gIZsau35EInIL25EfIqQdcrI6aQPOO4mqYmlVeypx7n94qrZaRjo/k7eU9Jia";
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);
    }

    public Bitmap getBitmap() throws InterruptedException{
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        Image rgb = frame.getImage(1);

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            int fmt = frame.getImage(i).getFormat();

            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;

            }
            else {
                opMode.telemetry.addLine("Didn't find correct rgb format");
                opMode.telemetry.update();

            }

        }

        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        frame.close();

        opMode.telemetry.addLine("Got Bitmap");
        opMode.telemetry.update();

        opMode.sleep(500);

        return bm;
    }

    public String sample() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        ArrayList<Integer> yValues = new ArrayList<>();

        int avgY = 0;

        //top left = (0,0)
        for (int rowNum = 0; rowNum < bitmap.getHeight(); rowNum ++) {

            for (int colNum = 200 ; colNum < 600; colNum ++) {
                int pixel = bitmap.getPixel(colNum, rowNum);

                int redPixel = red(pixel);
                int greenPixel = green(pixel);
                int bluePixel = blue(pixel);

                if (redPixel >= RED_THRESHOLD && greenPixel >= GREEN_THRESHOLD && bluePixel <= BLUE_THRESHOLD) {
                    yValues.add(rowNum);

                }

            }

        }

        for (int y : yValues) {
            avgY+= y;

        }

        opMode.telemetry.addData("Num Pixels found", yValues.size());
        opMode.telemetry.update();

        try {
            avgY /= yValues.size();
        } catch (ArithmeticException E){
            return  "posA";

        }

        opMode.telemetry.addData("avgY = ", avgY);
        opMode.telemetry.update();
        opMode.sleep(1000);


        if (avgY < 160) {
            return "posC";

        }
        else if (avgY < 200) {
            return "posB";

        }
        else {
            return "posA";

        }

    }

}