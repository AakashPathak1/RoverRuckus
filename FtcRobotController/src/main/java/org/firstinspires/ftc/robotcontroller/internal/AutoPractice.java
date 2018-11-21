package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


//@Autonomous(name="AutoPractice", group="Pushbot")
public class AutoPractice extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //double gyroReset=0;


    static final double     TURN_SPEED              = 0.6;     // Nominal half speed for better accuracy.
    static final double     MINIMUM_TURN_SPEED      = 0.2;
    static final double     DRIVE_SPEED             = 0.2;
    static final double APPROACH_SPEED = 0.2;
    static final double SPEED_FACTOR = 1.0;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime     runtime = new ElapsedTime();


    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    @Override
    public void runOpMode() {
        double headingAngle;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.useExternalCrystal = true;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //composeTelemetry();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //gravity  = imu.getGravity();
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //BEGIN PROGRAM

        waitForStart();
//        gyroDrive(DRIVE_SPEED, 24, 0, 10);
//        gyroTurn(MINIMUM_TURN_SPEED, 90,  70, 7);
        gyroSideDrive(DRIVE_SPEED, -10, 0);
        sleep(3000);
//        telemetry.addData("Heading:", angles.firstAngle);
//        telemetry.update();
//        sleep(3000);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading:", angles.firstAngle);
        telemetry.update();
        sleep(1000);
        gyroTurn(MINIMUM_TURN_SPEED, 0, 0, 10);
        sleep(5000);
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

        if(encoderCount > 0 && (runtime.seconds() < timeout)) {

            while (leftRear.getCurrentPosition() < (encoderCount + startPosition)) {

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

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        else if(encoderCount < 0 && (runtime.seconds() < timeout)) {

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
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

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
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

        if(encoderCount > 0 && (runtime.seconds() < timeout)) {

            while (leftRear.getCurrentPosition() < (encoderCount + startPosition)) {

                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);

                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }
        }

        else if(encoderCount < 0 && (runtime.seconds() < timeout)) {

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {


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

        if (targetAngle < 0 && (runtime.seconds() < timeout)){

            while (headingAngle >= (targetAngle + threshold)  || headingAngle <= (targetAngle - threshold)) {

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

        if (targetAngle > 0 && (runtime.seconds() < timeout)){

            while (headingAngle <= (targetAngle - threshold)  || headingAngle >= (targetAngle + threshold)) {
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

        if (targetAngle == 0.0 && (runtime.seconds() < timeout)){
            headingAngle = angles.firstAngle;
            while (headingAngle <= (targetAngle - threshold)  || headingAngle >= (targetAngle + threshold)) {
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

                headingAngle = angles.firstAngle;
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
//    public void gyroTurn (double speed, double targetAngle, double startSpeedCorrection, double timeout) {
//
//        double threshold=.2;
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double headingAngle = angles.firstAngle/*+gyroReset*/;
//
//        double currentSpeed = 0.25;
//        if(targetAngle>0) {
//            while ((headingAngle < (targetAngle - threshold) || headingAngle > (targetAngle + threshold) && opModeIsActive())) {
//                if (headingAngle < (targetAngle - threshold)) {
//                    leftFront.setPower(currentSpeed);
//                    rightFront.setPower(-currentSpeed);
//                    leftRear.setPower(currentSpeed);
//                    rightRear.setPower(-currentSpeed);
//                } else {
//                    leftFront.setPower(-currentSpeed);
//                    rightFront.setPower(currentSpeed);
//                    leftRear.setPower(-currentSpeed);
//                    rightRear.setPower(currentSpeed);
//                }
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                headingAngle = angles.firstAngle;
//                if (Math.abs(headingAngle - targetAngle) < (targetAngle - startSpeedCorrection)) {
//                    if (currentSpeed > 0.13) {
//                        leftFront.setPower(0);
//                        rightFront.setPower(0);
//                        leftRear.setPower(0);
//                        rightRear.setPower(0);
//                        sleep(1000);
//                    }
//                    currentSpeed = 0.125;
//                }
//
//                leftFront.setPower(currentSpeed);
//                rightFront.setPower(-currentSpeed);
//                leftRear.setPower(currentSpeed);
//                rightRear.setPower(-currentSpeed);
//            }
//        }
//        else{
//            while ((headingAngle < (targetAngle - threshold) || headingAngle > (targetAngle + threshold) && opModeIsActive())) {
//                if (headingAngle < (targetAngle - threshold)) {
//                    leftFront.setPower(-currentSpeed);
//                    rightFront.setPower(currentSpeed);
//                    leftRear.setPower(-currentSpeed);
//                    rightRear.setPower(currentSpeed);
//                } else {
//                    leftFront.setPower(currentSpeed);
//                    rightFront.setPower(-currentSpeed);
//                    leftRear.setPower(currentSpeed);
//                    rightRear.setPower(-currentSpeed);
//                }
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                headingAngle = angles.firstAngle;
//                if (Math.abs(headingAngle - targetAngle) < (targetAngle - startSpeedCorrection)) {
//                    currentSpeed = 0.125;
//                }
//
//                leftFront.setPower(currentSpeed);
//                rightFront.setPower(-currentSpeed);
//                leftRear.setPower(currentSpeed);
//                rightRear.setPower(-currentSpeed);
//            }
//        }
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//        //gyroReset+=targetAngle;
//    }

    public void gyroSideDrive(double speed, double inches, double angle) {
        //-inches = left
        //+inches = right
        double leftSpeed;
        double rightSpeed;
        double headingAngle;

        double encoderCount = inches * COUNTS_PER_INCH;
        double target = angle;
        double error;
        double startPosition = leftRear.getCurrentPosition();

        if(encoderCount > 0) {

            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed);
            rightRear.setPower(-speed);

            while (leftRear.getCurrentPosition() < (encoderCount + startPosition)) {
                headingAngle = angles.firstAngle;
                error = headingAngle - target;
                if(Math.abs(error) > 0.5) {
                    speed = speed - (error) / 100;
                    speed = speed + (error) / 100;
                } else {
                    speed = speed;
                    speed = speed;
                }


                leftSpeed = Range.clip(speed, -1, 1);
                rightSpeed = Range.clip(speed, -1, 1);



                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();

            }
        }

        else if(encoderCount < 0) {

            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftRear.setPower(-speed);
            rightRear.setPower(speed);

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
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



                telemetry.addData("encoder value", leftRear.getCurrentPosition());
                telemetry.update();
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        gyroTurn(MINIMUM_TURN_SPEED, angle, angle, 7);


        sleep(50);
    }


    void composeTelemetry() {
        telemetry.addAction(new Runnable() { @Override public void run()
        {
//            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });/*
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
                */
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
