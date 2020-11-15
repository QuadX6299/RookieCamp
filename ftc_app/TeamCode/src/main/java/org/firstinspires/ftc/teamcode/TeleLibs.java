package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class TeleLibs extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor intake;
    private DcMotor intake2;
    private DcMotor transition;
    private DcMotor shooter;
    private Servo wobble;
    private Servo claw;
    private Servo gate;

    private boolean shootSwitch;
    @Override
    public void init() {
        // MOTOR INITIALZATION
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        wobble = hardwareMap.servo.get("wobble");
        claw = hardwareMap.servo.get("claw");
        gate = hardwareMap.servo.get("gate");
        intake = hardwareMap.dcMotor.get("vacuum");
        intake2 = hardwareMap.dcMotor.get("vacuum2");
       transition = hardwareMap.dcMotor.get("shooter");
        shooter = hardwareMap.dcMotor.get("input");

        //actuator = hardwareMap.dcMotor.get("actuator");
        //output = hardwareMap.dcMotor.get("output");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }




        // =======================================  DRIVE  =============================================

    public void Drive() {
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        fl.setPower(leftPower);
        bl.setPower(leftPower);
        fr.setPower(rightPower);
        br.setPower(rightPower);
    }

    // =======================================  SHOOTER  ============================================

    public void shooter() {
        if(gamepad1.a) {
            shooter.setPower(0.5);
        } else if(gamepad1.b){
            shooter.setPower(0.6);
        } else if(gamepad1.x){
            shooter.setPower(0.73);
        } else if(gamepad1.y){
            shooter.setPower(0.72);
        } else{
            shooter.setPower(0);
        }
    }

    // ====================================== INTAKE =============================================
    public void intake () {
        intake.setPower(gamepad1.left_trigger);
        intake2.setPower(gamepad1.left_trigger);
    }
    // ======================================= TRANSITION =========================================
    public void transition(){
        transition.setPower(gamepad1.right_trigger);
    }
    // ====================================== wobble goal pick up =================================
    public void wobble(){
        if(gamepad1.dpad_down) {
            gate.setPosition(0);

        }else if(gamepad1.dpad_up){
            gate.setPosition(1);

        }

        if(gamepad1.dpad_left) {
            claw.setPosition(0);
        } else if(gamepad1.dpad_right){
            claw.setPosition(1);
        }
    }
}
    // https://github.com/rohitchawla28/LactoseIntolerant_/blob/master/ftc_app-master/TeamCode/src/main/java/LactoseIntolerant/Motion.java