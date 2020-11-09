package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class input {

    LinearOpMode opMode;
    public DcMotor intake;
    ElapsedTime timer;
    HardwareMap hardwareMap;


    public input(HardwareMap hwmap) {
        HardwareMap hardwareMap  = hwmap;
        intake = hardwareMap.get(DcMotor.class, "input");

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void input(double power) {

        if (power > 1) {
            power = 1;
        }
        intake.setPower(power);

    }

    public void stop() {
        intake.setPower(0);
    }

    public void shootSpeed(float bumper) {
//        timer = new ElapsedTime;
//        timer.start();
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double shootPower = 0;
        double error = 0;
        double time = 1;
        double change = 1;
        double proportional = 0;
        double integral;
        int oldTime = 0;
        int currTime = 0;
        int oldEncode = 0;

        while (Math.abs(bumper) != 0) {
//            currTime = timer.milliseconds();
//            //one rev is 28 tics
//            //circumfrence - tics to do one rev / rev
//            // power set to one not = one
//            //fix power - time to one rotation
//            oldEncode = wheelShoot.getCurrentPosition();
//            //error = (28/(wheelShoot.getCurrEncoder() - oldEncode)) / (time/currTime-oldTime);
//            error = (wheelShoot.getCurrentPosition() - oldEncode)/(time/currTime-oldTime);
//            proportional = error*kp;
//            shootPower = proportional+integral;
//            oldTime = currTime;
//
//            shoot(shootPower);

            intake.setPower(0.7);
        }

        stop();

    }

}
