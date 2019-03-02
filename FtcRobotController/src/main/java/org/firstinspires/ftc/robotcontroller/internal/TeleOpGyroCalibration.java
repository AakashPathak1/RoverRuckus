package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "Gyro Calibration", group = "8872")
public class TeleOpGyroCalibration extends LinearOpMode {
    //Drive motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    boolean slowMode = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.
    static final double     MINIMUM_TURN_SPEED      = 0.1;
    static final double     DRIVE_SPEED             = 0.4;
    static final double APPROACH_SPEED = 0.2;
    static final double SPEED_FACTOR = 1.0;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    //  Neverest 20 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        double frontWheelConstant = 1.15;

        double headingAngle;
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //composeTelemetry();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        waitForStart();
        // Reset enoders to zero
        while (opModeIsActive()) {

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
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


            //The negative signs are used to change the back of the robot to the front.

            if (slowMode) {
                leftFront.setPower(-v1 / 2 * frontWheelConstant);
                rightFront.setPower(-v2 / 2 * frontWheelConstant);
                leftRear.setPower(-v3 / 2);
                rightRear.setPower(-v4 / 2);
            } else {
                leftFront.setPower(-v1 * frontWheelConstant);
                rightFront.setPower(-v2 * frontWheelConstant);
                leftRear.setPower(-v3);
                rightRear.setPower(-v4);
            }

            if (gamepad1.a) {
                slowMode = true;
            }

            if (gamepad1.b) {
                slowMode = false;
            }

            if (gamepad1.dpad_up) {
                gyroDrive(DRIVE_SPEED, 12, 0, 5);
            }
            if (gamepad1.dpad_down) {
                gyroDrive(DRIVE_SPEED, -12, 0, 5);
            }
            if (gamepad1.dpad_right) {
                gyroSideDrive(0.6, 18, 0, 5);
            }
            if (gamepad1.dpad_left) {
                gyroSideDrive(0.6, -18, 0, 5);
            }

            telemetry.addData("rf: ", rightFront.getPower());
            telemetry.addData("lf: ", leftFront.getPower());
            telemetry.addData("rr: ", rightRear.getPower());
            telemetry.addData("lr: ", leftRear.getPower());
            telemetry.addData("encoder value LR", leftRear.getCurrentPosition());
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            headingAngle = angles.firstAngle;

            telemetry.addData("headingAngle", headingAngle);
            telemetry.addData("Slow Mode : ", slowMode);
            telemetry.update();


        }

    }

    public void gyroDrive(double speed, double inches, double angle, double timeout) {
        double leftSpeed;
        double rightSpeed;
        double headingAngle;

        double encoderCount = inches * COUNTS_PER_INCH;
        double target = angle;
        double error;
        double startPosition = leftRear.getCurrentPosition();

        runtime.reset();

        if(encoderCount > 0) {

            while ((leftRear.getCurrentPosition() < (encoderCount + startPosition))) {
                if (!opModeIsActive()) {
                    return;
                }
                headingAngle = angles.firstAngle;
                error = headingAngle - target;
                if(Math.abs(error) > 0.5) {
                    leftSpeed = speed - (error) / 100;
                    rightSpeed = speed + (error) / 100;
                } else {
                    leftSpeed = speed;
                    rightSpeed = speed;
                }


                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                leftFront.setPower(leftSpeed);
                rightFront.setPower(rightSpeed);
                leftRear.setPower(leftSpeed);
                rightRear.setPower(rightSpeed);

                telemetry.addData("encoder value LR", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        else if(encoderCount < 0) {

            while ((leftRear.getCurrentPosition() > (encoderCount + startPosition))) {
                if (!opModeIsActive()) {
                    return;
                }
                headingAngle = angles.firstAngle;
                error = headingAngle - target;

                if (Math.abs(error) > 0.5) {
                    leftSpeed = speed + (error) / 100;
                    rightSpeed = speed - (error) / 100;
                } else {
                    leftSpeed = speed;
                    rightSpeed = speed;
                }

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                leftFront.setPower(-leftSpeed);
                rightFront.setPower(-rightSpeed);
                leftRear.setPower(-leftSpeed);
                rightRear.setPower(-rightSpeed);

                telemetry.addData("encoder value LR", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        gyroTurn(MINIMUM_TURN_SPEED, angle,  angle, 7);

    }

    public void gyroDrive(double speed, double inches, double timeout) {

        double encoderCount = inches * COUNTS_PER_INCH;
        double startPosition = leftRear.getCurrentPosition();

        runtime.reset();

        if(encoderCount > 0) {

            while (leftRear.getCurrentPosition() < (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }

                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        else if(encoderCount < 0) {

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }

                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftRear.setPower(-speed);
                rightRear.setPower(-speed);

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);


        sleep(50);
    }


    public void gyroTurn (double speed, double targetAngle, double startSpeedCorrection, double timeout) {
        double currentSpeed;
        double headingAngle = angles.firstAngle;
        double error = Math.abs(targetAngle - headingAngle);
        //int counter = 0;
        double startJerk = 10;
        currentSpeed = Math.abs((error / targetAngle) * speed);

        double threshold = 0.5;

        runtime.reset();

        if (targetAngle < 0){

            while (headingAngle >= (targetAngle + threshold)  || headingAngle <= (targetAngle - threshold)) {
                if (!opModeIsActive()) {
                    return;
                }
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                headingAngle = angles.firstAngle;
                error = Math.abs(targetAngle - headingAngle);

                if (error < startSpeedCorrection) {
                    currentSpeed = Math.abs((error / targetAngle) * speed);
                }

                if (currentSpeed < MINIMUM_TURN_SPEED){
                    currentSpeed = MINIMUM_TURN_SPEED;
                }

                if (headingAngle >= (targetAngle + threshold)) {
                    leftFront.setPower(-currentSpeed);
                    rightFront.setPower(currentSpeed);
                    leftRear.setPower(-currentSpeed);
                    rightRear.setPower(currentSpeed);

                }

                if (headingAngle <= (targetAngle - threshold)) {
                    leftFront.setPower(currentSpeed);
                    rightFront.setPower(-currentSpeed);
                    leftRear.setPower(currentSpeed);
                    rightRear.setPower(-currentSpeed);
                }



                //sleep(50);

                if(error < startJerk) {

                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftRear.setPower(0);
                    rightRear.setPower(0);
                }

                //sleep(10);

                headingAngle = angles.firstAngle;
                //if (counter % 5 == 0){
                telemetry.addData("Heading ", headingAngle);
                //telemetry.addData("counter: ", counter);
                telemetry.update();
                //}
                //counter++;
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Update telemetry & Allow time for other processes to run.
            //if(counter % 1 == 0){
            telemetry.addData("angles:", angles.firstAngle);
            telemetry.update();
            //}

        }

        if (targetAngle > 0){

            while (headingAngle <= (targetAngle - threshold)  || headingAngle >= (targetAngle + threshold)) {
                if (!opModeIsActive()) {
                    return;
                }
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                headingAngle = angles.firstAngle;
                error = Math.abs(targetAngle - headingAngle);

                if (error < startSpeedCorrection) {
                    currentSpeed = Math.abs((error / targetAngle) * speed);
                }


                if (currentSpeed < MINIMUM_TURN_SPEED){
                    currentSpeed = MINIMUM_TURN_SPEED;
                }
                if(headingAngle <= -90){
                    headingAngle += 360;
                }

                if (headingAngle <= (targetAngle - threshold)) {
                    leftFront.setPower(currentSpeed);
                    rightFront.setPower(-currentSpeed);
                    leftRear.setPower(currentSpeed);
                    rightRear.setPower(-currentSpeed);

                }

                if (headingAngle >= (targetAngle + threshold)) {
                    leftFront.setPower(-currentSpeed);
                    rightFront.setPower(currentSpeed);
                    leftRear.setPower(-currentSpeed);
                    rightRear.setPower(currentSpeed);
                }



                // sleep(50);

                if(error < startJerk) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftRear.setPower(0);
                    rightRear.setPower(0);
                }

                //sleep(10);


                //if (counter % 1 == 0){
                telemetry.addData("Heading ", headingAngle);
                //    telemetry.addData("counter: ", counter);
                telemetry.update();
                //}
                //counter++;

            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);





            // Update telemetry & Allow time for other processes to run.
            //if(counter % 5 == 0){
            telemetry.update();
            //}
        }

        if (targetAngle == 0.0){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = angles.firstAngle;
            while (headingAngle <= (targetAngle - threshold)  || headingAngle >= (targetAngle + threshold)) {
                if (!opModeIsActive()) {
                    return;
                }
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                headingAngle = angles.firstAngle;
                error = targetAngle - headingAngle;

                if(error < 0) {
                    leftFront.setPower(-speed);
                    rightFront.setPower(speed);
                    leftRear.setPower(-speed);
                    rightRear.setPower(speed);
                } else if (error > 0){
                    leftFront.setPower(speed);
                    rightFront.setPower(-speed);
                    leftRear.setPower(speed);
                    rightRear.setPower(-speed);
                }

                //if (counter % 1 == 0){
                telemetry.addData("Heading ", headingAngle);
                //telemetry.addData("counter: ", counter);
                telemetry.update();
                //}
                //counter++;
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            telemetry.update();
        }
    }


    public void gyroSideDrive(double speed, double inches, double angle, double timeout) {
        //-inches = left
        //+inches = right
        double leftSpeed;
        double rightSpeed;
        double headingAngle;

        double rearWheelConstant = 1;
        double encoderCount = inches * COUNTS_PER_INCH;
        double target = angle;
        double error;
        double startPosition = leftRear.getCurrentPosition();

        runtime.reset();

        if(encoderCount > 0) {

            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed*rearWheelConstant);
            rightRear.setPower(-speed*rearWheelConstant);

            while (leftRear.getCurrentPosition() < (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }
//                headingAngle = angles.firstAngle;
//                error = headingAngle - target;
//                if(Math.abs(error) > 0.5) {
//                    speed = speed - (error) / 100;
//                    speed = speed + (error) / 100;
//                } else {
//                    speed = speed;
//                    speed = speed;
//                }


//                leftSpeed = Range.clip(speed, -1, 1);
//                rightSpeed = Range.clip(speed, -1, 1);



                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();

            }
        }

        else if(encoderCount < 0) {

            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftRear.setPower(-speed*rearWheelConstant);
            rightRear.setPower(speed*rearWheelConstant);

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
                if (!opModeIsActive()) {
                    return;
                }
//                headingAngle = angles.firstAngle;
//                error = headingAngle - target;
//
//                if (Math.abs(error) > 0.5) {
//                    leftSpeed = speed + (error) / 100;
//                    rightSpeed = speed - (error) / 100;
//                } else {
//                    leftSpeed = speed;
//                    rightSpeed = speed;
//                }
//
//                leftSpeed = Range.clip(leftSpeed, -1, 1);
//                rightSpeed = Range.clip(rightSpeed, -1, 1);



                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(150);
        gyroTurn(MINIMUM_TURN_SPEED, angle, angle, 7);


        sleep(50);
    }
}