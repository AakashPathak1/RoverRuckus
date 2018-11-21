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

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcontroller.internal.ThreeBlockYellowVision;
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

import java.sql.Driver;
import java.util.List;
import java.util.Locale;


import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="newAutoSideDrive", group="Pushbot")
public class newAutoSideDrive extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.
    static final double     MINIMUM_TURN_SPEED      = 0.2;
    static final double     DRIVE_SPEED             = 0.3;
    static final double APPROACH_SPEED = 0.2;
    static final double SPEED_FACTOR = 1.0;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    //  Neverest 20 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime     runtime = new ElapsedTime();

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;


    DcMotor arm1;
    DcMotor arm2;
    DcMotor scoopLifter;

    Servo leftLatch;
    Servo rightLatch;
    Servo marker;
    int armPos;
    int armPos2;

    boolean testMode = false;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private ThreeBlockYellowVision yellowVision;




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
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        scoopLifter = hardwareMap.dcMotor.get("scoopLifter");


        leftLatch = hardwareMap.servo.get("leftLatch");
        rightLatch = hardwareMap.servo.get("rightLatch");
        marker = hardwareMap.servo.get("marker");

        telemetry.update();

        while (!isStopRequested() && imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoopLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        headingAngle = angles.firstAngle;

        leftLatch.setPosition(1);
        rightLatch.setPosition(0.0);
        marker.setPosition(0);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.update();

        yellowVision = new ThreeBlockYellowVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        yellowVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//            yellowVision.setShowCountours(false);
        // start the vision system
        yellowVision.enable();

        while (!isStarted()) {
            leftLatch.setPosition(1);
            rightLatch.setPosition(0.0);
            marker.setPosition(0);

            scoopLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            headingAngle = angles.firstAngle;
//
//            telemetry.addData("headingAngle", headingAngle);


            // update the settings of the vision pipeline
            yellowVision.setShowCountours(true);

            // get a list of contours from the vision system
            List<MatOfPoint> contours = yellowVision.getContours();
            List<MatOfPoint> contourOne = yellowVision.getContourOne();
            boolean seesBlock;
            for (int i = 0; i < contourOne.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(contourOne.get(i));
                telemetry.addData("x-coordinate", (boundingRect.x + boundingRect.width) / 2);
                telemetry.addData("area", Imgproc.contourArea(contourOne.get(i)));
            }
            if (contourOne.size() > 0) {
                seesBlock = true;
            } else {
                seesBlock = false;
            }
            telemetry.addData("Sees Yellow Block", seesBlock);
            telemetry.update();




        }
//        Begin Program
       if (checkGold()) {
            gyroSideDrive(DRIVE_SPEED, -15, 0, 10);
            sleep(1000);
            gyroSideDrive(DRIVE_SPEED, 15, 0, 10);
           sleep(1000);

       }
        sleep(1000);

        gyroDrive(DRIVE_SPEED, 12, 0, 10);
        sleep(1000);


        if (checkGold()) {
            gyroSideDrive(DRIVE_SPEED, -15, 0, 10);
            sleep(1000);
            gyroSideDrive(DRIVE_SPEED, 15, 0, 10);
            sleep(1000);

        }
        sleep(1000);

        gyroDrive(DRIVE_SPEED, 12, 0, 10);
        sleep(1000);


        if (checkGold()) {
            sleep(1000);

            gyroSideDrive(DRIVE_SPEED, -15, 0, 10);
            sleep(1000);
            gyroSideDrive(DRIVE_SPEED, 15, 0, 10);
        }

        yellowVision.disable();

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

                telemetry.addData("encoder value LR", leftRear.getCurrentPosition());
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


    // Right is negative, left is positive
    public void gyroSideDrive(double speed, double inches, double angle, double timeout) {
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

        if(encoderCount > 0 && (runtime.seconds() < timeout)) {

            leftFront.setPower(-speed*frontWheelConstant);
            rightFront.setPower(speed*frontWheelConstant);
            leftRear.setPower(speed);
            rightRear.setPower(-speed);

            while ((leftRear.getCurrentPosition() < (encoderCount + startPosition)) && opModeIsActive()) {
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

        else if(encoderCount < 0 && (runtime.seconds() < timeout)) {

            leftFront.setPower(speed*frontWheelConstant);
            rightFront.setPower(-speed*frontWheelConstant);
            leftRear.setPower(-speed);
            rightRear.setPower(speed);

            while ((leftRear.getCurrentPosition() > (encoderCount + startPosition) && opModeIsActive())) {
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

        if(encoderCount > 0 && (runtime.seconds() < timeout)) {

            leftFront.setPower(-speed*frontWheelConstant);
            rightFront.setPower(speed*frontWheelConstant*1.2);
            leftRear.setPower(speed*1.2);
            rightRear.setPower(-speed);

            while ((leftRear.getCurrentPosition() < (encoderCount + startPosition) && opModeIsActive())) {
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

        else if(encoderCount < 0 && (runtime.seconds() < timeout)) {

            leftFront.setPower(speed*frontWheelConstant*1.2);
            rightFront.setPower(-speed*frontWheelConstant);
            leftRear.setPower(-speed);
            rightRear.setPower(speed*1.2);

            while ((leftRear.getCurrentPosition() > (encoderCount + startPosition) && opModeIsActive())) {
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

    public void armMove(int position, double speed, double timeout) {
        runtime.reset();

        double startPosition = arm1.getCurrentPosition();

        if(position > 0) {

            arm1.setPower(speed);
            arm2.setPower(speed);

            while ((arm1.getCurrentPosition() < (position + startPosition)) && (runtime.seconds() < timeout)) {

                telemetry.addData("encoder value arm", arm1.getCurrentPosition());
                telemetry.update();

            }
        }

        else if(position < 0) {

            arm1.setPower(-speed);
            arm2.setPower(-speed);

            while (arm1.getCurrentPosition() > (position + startPosition)&& (runtime.seconds() < timeout)) {

                telemetry.addData("encoder value arm", arm1.getCurrentPosition());
                telemetry.update();
            }
        }

        arm1.setPower(0);
        arm2.setPower(0);
    }

    public boolean checkGold() {
        //yellowVision.enable();
        // update the settings of the vision pipeline
        yellowVision.setShowCountours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = yellowVision.getContours();
        List<MatOfPoint> contourOne = yellowVision.getContourOne();
        boolean seesBlock;
        for (int i = 0; i < contourOne.size(); i++) {
            telemetry.addData("area", Imgproc.contourArea(contourOne.get(i)));
        }
        if (contourOne.size() > 0) {
            seesBlock = true;
        } else {
            seesBlock = false;
        }
        telemetry.addData("Sees Yellow Block", seesBlock);

        //yellowVision.disable();

        return seesBlock;
    }

    public void straightenGold(double speed) {
        //yellowVision.enable();
        // update the settings of the vision pipeline
        double frontWheelConstant = 1.15;
        yellowVision.setShowCountours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = yellowVision.getContours();
        List<MatOfPoint> contourOne = yellowVision.getContourOne();
        int coordinate;

        Rect boundingRect = Imgproc.boundingRect(contourOne.get(0));
        coordinate = (boundingRect.x + boundingRect.width) / 2;

        while((coordinate < 120) && opModeIsActive()) {
            contourOne = yellowVision.getContourOne();
            if (contourOne.size() > 0) {
                boundingRect = Imgproc.boundingRect(contourOne.get(0));
            }
            coordinate = (boundingRect.x + boundingRect.width) / 2;
            telemetry.addData("x-coordinate", coordinate);
            leftFront.setPower(-speed*frontWheelConstant);
            rightFront.setPower(speed*frontWheelConstant);
            leftRear.setPower(speed);
            rightRear.setPower(-speed);
        }

        while ((coordinate > 160) && opModeIsActive()) {
            contourOne = yellowVision.getContourOne();
            if (contourOne.size() > 0) {
                boundingRect = Imgproc.boundingRect(contourOne.get(0));
            }
            coordinate = (boundingRect.x + boundingRect.width) / 2;
            telemetry.addData("x-coordinate", coordinate);
            leftFront.setPower(speed*frontWheelConstant);
            rightFront.setPower(-speed*frontWheelConstant);
            leftRear.setPower(-speed);
            rightRear.setPower(speed);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //yellowVision.disable();

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

