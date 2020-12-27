package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain {

    private LinearOpMode newOp;
    private GyroCode sensors;

    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorFR;
    private DcMotor motorBR;

    public void init(LinearOpMode opMode) throws InterruptedException {
        newOp = opMode;

        motorFL = opMode.hardwareMap.dcMotor.get("fl");
        motorBL = opMode.hardwareMap.dcMotor.get("bl");
        motorFR = opMode.hardwareMap.dcMotor.get("fr");
        motorBR = opMode.hardwareMap.dcMotor.get("br");

        sensors = new GyroCode(opMode);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    }

    public void resetEncoders() {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();

    }

    public double getAvgEncoder() {

        double div = 4;
        double avgAdd = motorBL.getCurrentPosition() + motorBR.getCurrentPosition() + motorFL.getCurrentPosition() + motorFR.getCurrentPosition();
        double avg;

        if (motorBL.getCurrentPosition() == 0) {
            div--;
        }

        if (motorBR.getCurrentPosition() == 0) {
            div--;
        }
        if (motorFL.getCurrentPosition() == 0) {
            div--;
        }

        if (motorFR.getCurrentPosition() == 0) {
            div--;
        }

        if (div == 0) {
            avg = 0;
        } else {
            avg = avgAdd / div;
        }

        return avg;
    }

    public void straight(double pwr, double RhAdj, double LhAdj) {

        double max = Math.max(Math.abs(pwr + LhAdj), Math.abs(pwr + RhAdj));
        double leftPwr = pwr + LhAdj;
        double rightPwr = pwr + RhAdj;

        if (max > 1) {
            leftPwr /= max;
            rightPwr /= max;
        }

        motorFL.setPower(leftPwr);
        motorFR.setPower(rightPwr);
        motorBL.setPower(leftPwr);
        motorBR.setPower(rightPwr);
    }

    public void stopAll() {

        double pwr = 0;
        motorFL.setPower(pwr);
        motorFR.setPower(pwr);
        motorBL.setPower(pwr);
        motorBR.setPower(pwr);

    }

    public GyroCode getSensors() {
        return sensors;
    }

    public void goStraightPID(double distance, double kP, double kI, double kD, double timeout, double max) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double oldGyro = sensors.getCurrGyro();
        double power;
        double error = 0;
        double currTime = timer.milliseconds();
        double LhAdjust = 0;
        double RhAdjust = 0;
        double integral = 0;
        double derivative;
        double proportional;
        double oldTime = currTime;


        while (Math.abs(distance) > Math.abs(getAvgEncoder()) && !newOp.isStopRequested()) {
            currTime = timer.milliseconds();

            proportional = (distance - getAvgEncoder()) * kP;
            integral += (distance - getAvgEncoder()) * (currTime - oldTime) * kI;
            derivative = sensors.getTrueDiff(oldGyro) * kD;
            power = integral + proportional + derivative;

            error = sensors.getCurrGyro() - oldGyro;

            RhAdjust = -(error * .028);
            LhAdjust = (error * .028);

            if(power < 0.15 && distance > 0){
                power = 0.15;
            }
            if(power > -0.15 && distance < 0){
                power = -0.15;
            }

            if(Math.abs(power) > Math.abs(max)){
                power = max;
            }
            oldTime = currTime;
            straight(power, RhAdjust, LhAdjust);

            newOp.telemetry.addData("Avg Encoder Val", getAvgEncoder());
            newOp.telemetry.addData("Gyro Error", error);
            newOp.telemetry.addData("Forward power", power);
            newOp.telemetry.addData("Integral", integral);
            newOp.telemetry.addData("Left power: ", LhAdjust);
            newOp.telemetry.addData("Right power: ", RhAdjust);
            newOp.telemetry.update();
            if (currTime > timeout) {
                break;
            }
        }

        stopAll();
    }

    public void turn(double power) {
        motorFL.setPower(-power);
        motorFR.setPower(power);
        motorBL.setPower(-power);
        motorBR.setPower(power);
    }

    public void turn_right(double distance, double pwr) {
        int startVal = motorFL.getCurrentPosition();
        while (newOp.opModeIsActive() && (Math.abs(motorFL.getCurrentPosition() - startVal) < distance)) {
            motorFL.setPower(-pwr);
            motorFR.setPower(pwr);
            motorBL.setPower(-pwr);
            motorBR.setPower(pwr);
            newOp.telemetry.addData("turnRight", motorFL.getCurrentPosition());
            newOp.telemetry.addData("startValue", startVal);
            newOp.telemetry.update();
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    }

    public void gyroAdjust() {

        ElapsedTime timer1 = new ElapsedTime();
        double start_time = timer1.time();
        start_time =timer1.time();

        while(true&&timer1.time() <start_time +1.5) {
            sensors.angles = sensors.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            newOp.telemetry.addData("Heading2 ", sensors.angles.firstAngle);
            newOp.telemetry.update();
            if (sensors.angles.firstAngle > 0.5) {
                turn_left(10, 0.25);
            } else if (sensors.angles.firstAngle < -0.5) {
                turn_right(10, 0.25);
            } else {
                break;
            }
            newOp.sleep(50);
        }

    }

    public void turn_left(double distance, double pwr) {
        int startVal =  motorFL.getCurrentPosition();
        while(newOp.opModeIsActive() && (Math.abs(motorFL.getCurrentPosition() - startVal) < distance)) {
            motorFL.setPower(pwr);
            motorFR.setPower(-pwr);
            motorBL.setPower(pwr);
            motorBR.setPower(-pwr);
            newOp.telemetry.addData("turnLeft", motorFL.getCurrentPosition());
            newOp.telemetry.addData("startValue", startVal);
            newOp.telemetry.update();
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    }

    public void turnPID(double turnAng, double kP, double kI, double kD, boolean base) {

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        double power = 0;
        double oldGyro = sensors.getCurrGyro();
        double error = sensors.getTrueDiff(turnAng);
        double oldError;
        double currTime = timer.milliseconds();
        double oldTime = currTime;
        double proportional;
        double integral = 0;
        double derivative;


        while ((Math.abs(error) > 0.5 || Math.abs(power) > 0.2) && newOp.opModeIsActive()) {
            oldError = error;
            error = sensors.getTrueDiff(turnAng);

            currTime = timer.milliseconds();
            proportional = error * kP;
            integral += error * (currTime - oldTime) * kI;
            derivative = (error - oldError) / (currTime - oldTime) * kD;

            power = proportional + integral + derivative;

            if (error <= 0 && base) {
                power -= 0.15;
            } else if (base) {
                power += 0.15;
            }

            turn(power);

            newOp.telemetry.addData("Power", power);
            newOp.telemetry.addData("Proportional", proportional);
            newOp.telemetry.addData("Integral", integral);
            newOp.telemetry.addData("Derivative", derivative);
            newOp.telemetry.update();
            oldTime = currTime;

        }

        error = sensors.getTrueDiff(turnAng);
        newOp.telemetry.addData("Error", error);
        newOp.telemetry.update();

        stopAll();
    }

    public void shift(double power, double LhAdjust, double RhAdjust) {
        motorFL.setPower(-power + LhAdjust);
        motorFR.setPower(power + RhAdjust);
        motorBL.setPower(power + LhAdjust);
        motorFL.setPower(-power + RhAdjust);
    }

    public void shiftPID(double distance, double kP, double kI, double kD) {
        ElapsedTime timer = new ElapsedTime();
        resetEncoders();
        timer.startTime();

        double power;
        double oldError;
        double oldGyro = sensors.getCurrGyro();
        double error = sensors.getCurrGyro() - oldGyro;
        double currTime = timer.milliseconds();
        double LhAdjust;
        double RhAdjust;
        double integral = 0;
        double proportional;
        double derivative;
        double oldTime = currTime;


        while (Math.abs(distance) > Math.abs(getAvgEncoder()) && !newOp.isStopRequested()) {
            currTime = timer.milliseconds();
            oldError = error;
            error = sensors.getCurrGyro() - oldGyro;
            proportional = (distance - getAvgEncoder()) * kP;
            integral += (distance - getAvgEncoder()) * (oldTime - currTime) * kI;
            derivative = error - oldError / (currTime - oldTime) * kD;
            power = proportional + integral + derivative;


            RhAdjust = -(error * .013);
            LhAdjust = (error * .013);

            if (power < 0.15) {
                power += 0.13;
            }
            oldTime = currTime;

            shift(power, RhAdjust, LhAdjust);

            newOp.telemetry.addData("Avg Encoder Val", getAvgEncoder());
            newOp.telemetry.addData("Gyro Error", error);
            newOp.telemetry.addData("Forward power", power);
            newOp.telemetry.addData("Left power: ", LhAdjust);
            newOp.telemetry.addData("Right power: ", RhAdjust);
            newOp.telemetry.update();
        }

        stopAll();
        newOp.telemetry.addData("Gyro Error", sensors.getCurrGyro());
        newOp.telemetry.update();
        newOp.sleep(5000);

    }

    public void shift_left(double distance, double pwr)
    {
        int startVal = motorFR.getCurrentPosition();
        while (newOp.opModeIsActive() && ((Math.abs(motorFR.getCurrentPosition() - startVal))) <  distance ){
            // implement PID control here
            motorFL.setPower(-pwr);
            motorFR.setPower(pwr);
            motorBL.setPower(pwr);
            motorBR.setPower(-pwr);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);


    }

    public void shift_right( double distance, double pwr)
    {
        int startVal = motorFL.getCurrentPosition();
        while(newOp.opModeIsActive() && (Math.abs(motorFL.getCurrentPosition() - startVal) < distance)) {
            // implement PID control here
            motorFL.setPower(-pwr);
            motorFR.setPower(pwr);
            motorBL.setPower(pwr);
            motorBR.setPower(-pwr);

            //wrist_stay(wristStartVal, 100, -0.3);
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    }
}
