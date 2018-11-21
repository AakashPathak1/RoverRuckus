package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTest", group = "8872")
@Disabled
public class ServoTest extends LinearOpMode {
    //Drive servo
    Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {


        arm = hardwareMap.servo.get("arm");

        waitForStart();
        arm.setPosition(0.5);
        // Reset enoders to zero
        while (opModeIsActive()) {

            if (gamepad1.a) {
                arm.setPosition(1.0);
            }

            if (gamepad1.b) {
                arm.setPosition(0.0);
            }
        }

    }
//    public void turnArm() {
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        arm.setTargetPosition(1680);
//
//        armPos = arm.getCurrentPosition();
//
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(armPos < 1680){
//            arm.setPower(1);
//
//            armPos = arm.getCurrentPosition();
//        }
//
//        arm.setPower(0);
//        telemetry.addData("2 ", "arm:  " + String.format("%d", arm.getTargetPosition()));
//        arm.setPower(-0.05);
//
//    }
//
//    public void turnArmBack() {
//        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        arm.setTargetPosition(0);
//
//        armPos = arm.getCurrentPosition();
//
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(armPos > 0){
//            arm.setPower(-1);
//
//            armPos = arm.getCurrentPosition();
//        }
//
//        arm.setPower(0);
//        telemetry.addData("2 ", "arm:  " + String.format("%d", arm.getTargetPosition()));
//        arm.setPower(0.05);
//
//    }
}