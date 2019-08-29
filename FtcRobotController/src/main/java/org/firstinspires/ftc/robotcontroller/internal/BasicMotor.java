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

@TeleOp(name = "BasicMotor", group = "8872")
public class BasicMotor extends LinearOpMode {
    //Drive motors
    DcMotor motor;
    Servo leftCollect;
    Servo rightCollect;
    double motorPower;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        motor = hardwareMap.dcMotor.get("motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftCollect = hardwareMap.servo.get("leftCol");
        rightCollect = hardwareMap.servo.get("rightCol");

        waitForStart();


        // Reset enoders to zero

        while (opModeIsActive()) {

            motorPower = -gamepad1.left_stick_y;

            telemetry.addData("Motor Encoder Position: ", motor.getCurrentPosition());
            telemetry.addData("motor power", motor.getPower());
            telemetry.update();

            if(Math.abs(motorPower)<.05) {
                motor.setPower(motor.getCurrentPosition()/7000.0);
            } else {
                motor.setPower(motorPower);
            }

            if (gamepad2.a) {
                leftCollect.setPosition(0);
                rightCollect.setPosition(1);
            }

            if (gamepad2.b) {
                leftCollect.setPosition(1);
                rightCollect.setPosition(0);
            }

        }

    }
}

