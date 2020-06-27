package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class drivetrain {

    private LinearOpMode opMode;
    private Gyro gyro;

    private DcMotor frontLeft;
    private DcMotor middleLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor middleRight;
    private DcMotor backRight;

    public drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        gyro = new Gyro(opMode, true);

        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        middleLeft = opMode.hardwareMap.dcMotor.get("middleLeft");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        middleRight = opMode.hardwareMap.dcMotor.get("middleRight");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        middleLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        middleRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}