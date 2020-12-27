package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.Locale;

import static java.lang.Thread.sleep;

public class GyroCode {

    public BNO055IMU gyro;
    LinearOpMode opMode;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters;

    public GyroCode(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

    }

    public double getCurrGyro() {
        updateValues();
        return angles.firstAngle;
    }

    // returns positive number to turn left and negative number to turn right
    public double getTrueDiff(double destTurn){
        double currAng = getCurrGyro();
        if((currAng >= 0 && destTurn >= 0) || (currAng <= 0 && destTurn <= 0))
            return destTurn - currAng;
        else if(Math.abs(destTurn - currAng) <= 180)
            return destTurn - currAng;
        else if(destTurn > currAng)
            return -(360 - (destTurn - currAng));
        else
            return 360 - (currAng - destTurn);

    }
    public boolean resetGyro() {
        return gyro.initialize(parameters);
    }

    public void updateValues() {
        angles = gyro.getAngularOrientation();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}