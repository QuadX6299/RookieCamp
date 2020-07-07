package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TeleLibs extends OpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor intakeL;
    private DcMotor intakeR;

    @Override
    public void init() {
        // MOTOR INITIALZATION
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeR = hardwareMap.dcMotor.get("intakeR");
        
    // ________________________________________________________________________________________________
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
    }




        // =======================================  DRIVE  =============================================
   
    public void arcadeDrive() {
        //checking for valid range to apply power (has to give greater power than .1)
        if ((Math.abs(gamepad1.left_stick_y) > .1) ||
                Math.abs(gamepad1.right_stick_x) > .1) {

            if (Math.abs(left_stick_y > .1)){
                frontLeft.setPower(-gamepad1.left_stick_y);
                frontRight.setPower(-gamepad1.left_stick_y);
                backLeft.setPower(-gamepad1.left_stick_y);
                backRight.setPower(-gamepad1.left_stick_y); 
            }
            else {
                frontLeft.setPower(gamepad1.right_stick_x);
                frontRight.setPower(-gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
            }

        }
        else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }

    }
    }
    // https://github.com/rohitchawla28/LactoseIntolerant_/blob/master/ftc_app-master/TeamCode/src/main/java/LactoseIntolerant/Motion.java