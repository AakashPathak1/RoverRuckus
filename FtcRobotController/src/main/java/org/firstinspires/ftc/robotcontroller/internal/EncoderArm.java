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
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "EncoderArm", group = "8872")
public class EncoderArm extends LinearOpMode {
    //Drive motors
    DcMotor arm1;
    DcMotor arm2;
    int armPos;
    int armPos2;

    @Override
    public void runOpMode() throws InterruptedException {

        // opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");

        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        // Reset enoders to zero
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        arm1.setTargetPosition(1120);
        arm2.setTargetPosition(1120);


        armPos = arm1.getCurrentPosition();
        armPos2 = arm2.getCurrentPosition();

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while((armPos < 1120 || armPos2 < 1120) && opModeIsActive() ){
            arm1.setPower(0.5);
            arm2.setPower(0.5);

            telemetry.addData("arm1 power: ", arm1.getPower());
            armPos = arm1.getCurrentPosition();
        }


        arm1.setPower(0);
        arm2.setPower(0);
        telemetry.addData("2 ", "arm:  " + String.format("%d", arm1.getTargetPosition()));




    }
}