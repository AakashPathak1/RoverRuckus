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

@TeleOp(name = "8872 Final TeleOp", group = "8872")
public class TeleOpArmTest extends LinearOpMode {
    //Drive motors
    DcMotor arm1;
    DcMotor arm2;
    int armPos;
    int armPos2;
    double armPower;
    double scoopLifterPower;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor scoopLifter;
    Servo leftLatch;
    Servo rightLatch;
    Servo marker;
    Servo col;
    boolean slowMode = false;

    static final double APPROACH_SPEED = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private ElapsedTime     runtime = new ElapsedTime();

    double frontWheelConstant = 1.15;

    @Override
    public void runOpMode() throws InterruptedException {


        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        scoopLifter = hardwareMap.dcMotor.get("scoopLifter");
        leftLatch = hardwareMap.servo.get("leftLatch");
        rightLatch = hardwareMap.servo.get("rightLatch");
        marker = hardwareMap.servo.get("marker");
        col = hardwareMap.servo.get("col");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLatch.setPosition(0.05);
        rightLatch.setPosition(0.9);

        marker.setPosition(0);

        double frontWheelConstant = 1.15;


        waitForStart();
        // Reset enoders to zero
        while (opModeIsActive()) {


            arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armPower = gamepad2.right_stick_y;

            scoopLifterPower = gamepad2.left_stick_y;

            scoopLifterPower = Math.max(-0.5, scoopLifterPower);
            scoopLifterPower = Math.min(0.5, scoopLifterPower);

            scoopLifter.setPower(-scoopLifterPower);


            if (gamepad2.dpad_down) {
                col.setPosition(0);
            }

            if (gamepad2.dpad_up) {
                col.setPosition(1);
            }


            arm1.setPower(armPower);
            arm2.setPower(armPower);

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX =  -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
//            final double v1 = 1;
//            final double v2 = 1;
//            final double v3 = 1;
//            final double v4 = 1;

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            scoopLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            //The negative signs are used to change the back of the robot to the front.

            if (slowMode) {
                leftFront.setPower(-v1 / 2 * frontWheelConstant);
                rightFront.setPower(-v2 / 2 * frontWheelConstant);
                leftRear.setPower(-v3 / 2);
                rightRear.setPower(-v4 / 2);
            }
            else {
                leftFront.setPower(-v1 * frontWheelConstant);
                rightFront.setPower(-v2 * frontWheelConstant);
                leftRear.setPower(-v3);
                rightRear.setPower(-v4);
            }




            if (gamepad2.x) {
                leftLatch.setPosition(0.05);
                rightLatch.setPosition(0.9);
            }
            if (gamepad2.y) {
                leftLatch.setPosition(1);
                rightLatch.setPosition(0.0);
            }
            //calibrate marker values. TAKE THIS OUT WHEN MAKING FINAL TELEOP
            if (gamepad1.a) {
                marker.setPosition(0);
            }
            if (gamepad1.b) {
                marker.setPosition(1);
            }

            if (gamepad1.x) {
                pullUp(-4000, 1, 6);
                arm1.setPower(-0.5);
                arm2.setPower(-0.5);
                sleep(500);
                leftLatch.setPosition(1);
                rightLatch.setPosition(0.0);
                sleep(500);
                arm1.setPower(0);
                arm2.setPower(0);
            }

            if (gamepad1.a) {
                slowMode = true;
            }

            if (gamepad1.b) {
                slowMode = false;
            }

            telemetry.addData("rf: ", rightFront.getPower());
            telemetry.addData("lf: ", leftFront.getPower());
            telemetry.addData("rr: ", rightRear.getPower());
            telemetry.addData("lr: ", leftRear.getPower());

            telemetry.addData("Slow Mode : ", slowMode);
            telemetry.update();


        }

    }

    public void pullUp(int position, double speed, double timeout) {
        runtime.reset();


        double startPosition = arm1.getCurrentPosition();

        if(position > 0) {

            arm1.setPower(speed);
            arm2.setPower(speed);

            while ((arm1.getCurrentPosition() < (position + startPosition)) && (runtime.seconds() < timeout)) {

                telemetry.addData("encoder value", arm1.getCurrentPosition());
                telemetry.update();

            }
        }

        else if(position < 0) {

            arm1.setPower(-speed);
            arm2.setPower(-speed);

            while (arm1.getCurrentPosition() > (position + startPosition)&& (runtime.seconds() < timeout)) {

                telemetry.addData("encoder value", arm1.getCurrentPosition());
                telemetry.update();
            }
        }

        arm1.setPower(0);
        arm2.setPower(0);
    }
}