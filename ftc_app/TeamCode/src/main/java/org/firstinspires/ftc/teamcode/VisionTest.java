package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;

@Autonomous(name = "Vision Test Real", group = "Concept")
//@Disabled
public class VisionTest extends LinearOpMode {


    Vision vision;
    public ElapsedTime timer = new ElapsedTime();

    String skystonePosition = "";

    public VisionTest() throws InterruptedException {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /* Wait for the game to begin */

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        int counter = 0;

        telemetry.addLine("Initialized");
        telemetry.update();

        vision = new Vision(this);

        while (!isStarted()) {
            telemetry.addData("Count ", counter);
            counter++;
            telemetry.update();
            skystonePosition = vision.sample();
            telemetry.addData("Position", skystonePosition);
            // Display the light level while we are waiting to start
            //telemetry.addData("Light Level", lightSensor.getLightDetected());
            //telemetry.update();
            idle();
        }

        waitForStart();

        switch (skystonePosition) {
            case "posA":
                break;

            case "posB":
                break;

            case "posC":
                break;

        }

        telemetry.addData("Robot should be moving now", -1);
        telemetry.update();
//        drive.turnPID(90, .0075, .000011, 0.5, true); //best version - left 90 degree turn
//        sleep(3000);
//        drive.turnPID(135, 0.009, 0.000019, 0.8, true);//best version - left 45 degree turn
//        sleep(3000);
//        drive.turnPID(150, 0.007, 0.000019, 0.5, true);// best version - left 15 degree turn
        //sleep(3000);



    }



}