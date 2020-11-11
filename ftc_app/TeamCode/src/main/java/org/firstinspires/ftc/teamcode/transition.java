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

public class transition {

    LinearOpMode opMode;
    public DcMotor intake;
    ElapsedTime timer;
    HardwareMap hardwareMap;


    public transition(HardwareMap hwmap) {
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



}


