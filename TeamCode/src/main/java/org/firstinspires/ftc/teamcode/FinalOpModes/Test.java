package org.firstinspires.ftc.teamcode.FinalOpModes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@Autonomous(name="Test", group="Pushbot")
public class Test extends LinearOpMode {
    protected BNO055IMU imu;
    protected Orientation angles;
    protected Acceleration gravity;
    protected double headingAngle;
    protected static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.
    protected static final double     MINIMUM_TURN_SPEED      = 0.2;
    protected static final double     DRIVE_SPEED             = 0.3;
    protected static final double APPROACH_SPEED = 0.2;
    protected static final double SPEED_FACTOR = 1.0;
    protected static final double COUNTS_PER_MOTOR_REV = 537.6;    //  Neverest 20 Motor Encoder
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    protected ElapsedTime runtime = new ElapsedTime();

    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftRear;
    protected DcMotor rightRear;
    protected DcMotor scoopLifter;



    protected boolean testMode = false;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() {

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
        composeTelemetry();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);






        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");






        telemetry.update();

        while (!isStopRequested() && imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        headingAngle = angles.firstAngle;




        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.update();

        while (!isStarted()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            headingAngle = angles.firstAngle;

            telemetry.addData("headingAngle", headingAngle);







            telemetry.update();
        }
//      Begin Program

        telemetry.addData("PHONE SEES THIS", headingAngle);
        telemetry.update();
    }

    public void gyroTurnNew(double targetAngle) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double headingAngle = angles.firstAngle;
        double currentSpeed = 1;

        while (headingAngle < targetAngle) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = angles.firstAngle;
            leftFront.setPower(-currentSpeed);
            rightFront.setPower(currentSpeed);
            leftRear.setPower(-currentSpeed);
            rightRear.setPower(currentSpeed);
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

        sleep(100);

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

    public void gyroSideDrive(double speed, double inches, double timeout) {
        //-inches = left
        //+inches = right
        double leftSpeed;
        double rightSpeed;
        double headingAngle;

        double frontWheelConstant = 1.15;
        double encoderCount = inches * COUNTS_PER_INCH;
        double error;
        double startPosition = leftRear.getCurrentPosition();

        runtime.reset();

        if(encoderCount > 0) {

            leftFront.setPower(-speed*frontWheelConstant);
            rightFront.setPower(speed*frontWheelConstant);
            leftRear.setPower(speed);
            rightRear.setPower(-speed);

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

            leftFront.setPower(speed*frontWheelConstant);
            rightFront.setPower(-speed*frontWheelConstant);
            leftRear.setPower(-speed);
            rightRear.setPower(speed);

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

        sleep(50);
    }

    public void gyroSideDriveWall(double speed, double inches, double angle, double timeout) {
        //-inches = left
        //+inches = right
        double leftSpeed;
        double rightSpeed;
        double headingAngle;

        double frontWheelConstant = 1.15;
        double encoderCount = inches * COUNTS_PER_INCH;
        double target = angle;
        double error;
        double startPosition = leftRear.getCurrentPosition();

        runtime.reset();

        if(encoderCount > 0) {

            leftFront.setPower(-speed*frontWheelConstant);
            rightFront.setPower(speed*frontWheelConstant*1.2);
            leftRear.setPower(speed*1.2);
            rightRear.setPower(-speed);

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

            leftFront.setPower(speed*frontWheelConstant*1.2);
            rightFront.setPower(-speed*frontWheelConstant);
            leftRear.setPower(-speed);
            rightRear.setPower(speed*1.2);

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

        gyroTurn(MINIMUM_TURN_SPEED, angle, angle, 7);


        sleep(50);
    }

    public void resetGyro() {
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }







    /**
     * Initialize the Tensor Flow Object Detection engine.
     */


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

