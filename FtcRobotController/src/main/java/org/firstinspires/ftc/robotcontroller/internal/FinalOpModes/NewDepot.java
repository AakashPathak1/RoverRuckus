package org.firstinspires.ftc.robotcontroller.internal.FinalOpModes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;


@Autonomous(name="NewDepot", group="Pushbot")
public class NewDepot extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.
    static final double     MINIMUM_TURN_SPEED      = 0.2;
    static final double     DRIVE_SPEED             = 0.4;
    static final double APPROACH_SPEED = 0.2;
    static final double SPEED_FACTOR = 1.0;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    //  Neverest 20 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime     runtime = new ElapsedTime();

    ColorSensor sensorColor;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    DcMotor pullUp;

    Servo marker;
    Servo colLeft;
    Servo colRight;

    DigitalChannel digitalTouch;  // Hardware Device Object

    boolean testMode = false;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AZKGTZf/////AAABmaNQYUeIb0wZqzGIIEMZHY5LB+fIxOJ5Rs+lYn" +
            "dWRlWknEZzgXyjOYvsVg7iMPga7dhuKlPiGjczKoa+CiByVpadKZO1kb9BZn3aIfaEMIatKZ2cnPn2fTx12DgfI3" +
            "v5OyINq2YMKDN8FuE9NJP7g0vBHJPCEjr/nX4BG84RV1FUVlrgqWVOATwdkjRZp2hOVB+sQKDU13jDgMpNGKZya" +
            "S5F00Qc0snjcX7gBg9KTaXVig+juk2jg4yyoXyzC7wbpJzYZt0zuRmjvNlYWEtDi1fqCKudrqkIdUVZLL7QR590" +
            "oqQN3fenTWvdnuLh/InqsovkUfcxELZVzoYLHv1Pq17J7UUL8o3lvb8Ns5dsGXHq";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

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

        sensorColor = hardwareMap.get(ColorSensor.class, "color");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        pullUp = hardwareMap.dcMotor.get("pullUp");;

        marker = hardwareMap.servo.get("marker");
        colLeft = hardwareMap.servo.get("colLeft");
        colRight = hardwareMap.servo.get("colRight");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.update();

        while (!isStopRequested() && imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        pullUp.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        headingAngle = angles.firstAngle;

        marker.setPosition(0);

        colLeft.setPosition(0);
        colRight.setPosition(1);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pullUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.update();

        while (!isStarted()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            headingAngle = angles.firstAngle;

            telemetry.addData("headingAngle", headingAngle);

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            marker.setPosition(0);

            colLeft.setPosition(0);
            colRight.setPosition(1);

            telemetry.update();
        }
//      Begin Program
        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Drop down from Lander

        pullUp.setPower(-1.0);

        //runtime.reset();
        while (digitalTouch.getState() == true/* && (runtime.seconds() < 8.0)*/) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            sleep(10);
        }
        telemetry.addData("Digital Touch", "Is Pressed");

        pullUp.setPower(0.0);

        //pullUp(-9050, 1.0, 5);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Drive forward to remove hook
        gyroDrive(DRIVE_SPEED, -1, 0, 5);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Drive away from the lander
        gyroSideDrive(0.4, -8, 0, 10);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //start vision
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        tfod.activate();
        sleep(1000);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Create Adaptive Variables to keep track of bot location and task completion
        boolean hitGold = false;
        int position = 0;
        sleep(400);
        //Check if middle mineral is gold
        if (!hitGold && checkGold()) {
            //Hit the gold and come back
            gyroDrive(DRIVE_SPEED,2,0,10);
            sleep(200);
            gyroSideDrive(0.4, -18, 0, 10);
            sleep(200);
            gyroSideDrive(0.4, 9, 0, 10);

            position = 2;
            hitGold = true;
        } else {
            //Turn to the right mineral
            gyroTurn(TURN_SPEED, -42, -42, 10);
            sleep(800);

        }

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Check if right mineral is gold
        if (!hitGold && checkGold()) {
            //Hit the gold and come back
            gyroSideDrive(DRIVE_SPEED, -22, -42, 10);
            gyroSideDrive(DRIVE_SPEED, 9, -42, 10);

            position = 1;
            hitGold = true;
        } else {
            //Turn to the left mineral
//            if (!hitGold) {
//                gyroTurn(TURN_SPEED, 38, 38, 10);
//            }
        }

        //Check if left mineral is gold
//        if (!hitGold && checkGold()) {
//            //Turn more and Hit the gold and come back
//            gyroTurn(TURN_SPEED, 45, 45, 10);
//            gyroSideDrive(DRIVE_SPEED, -20, 45, 10);
//            gyroSideDrive(DRIVE_SPEED, 20, 45, 10);
//
//            position = 3;
//            hitGold = true;
//        }
        if (!hitGold) {
            gyroTurn(TURN_SPEED, 45, 45, 10);
            gyroSideDrive(DRIVE_SPEED, -26, 45, 10);
            gyroSideDrive(DRIVE_SPEED, 13, 45, 10);
        }

        tfod.shutdown();

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        gyroTurn(0.2, 0.0, 0.0, 10);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Drive to wall

        gyroDrive(DRIVE_SPEED,40, 0, 10);


        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Turn to -135 to ram

        gyroTurn(TURN_SPEED,-135,-135,10);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //RAM WALLLLLLLLLLLL
        gyroSideDrive(DRIVE_SPEED, 20, 10);
        sleep(500);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                // convert the RGB values to HSV values.
                // multiply by the SCALE_FACTOR.
                // then cast it back to int (SCALE_FACTOR is a double)
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();
                sleep(10);
            }
        }

        gyroDrive(DRIVE_SPEED, 10, 5);
        //Back up to Depot (Stop using light sensor)
        leftFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(0.5);

        //while (hsvValues[0] < 150 && hsvValues[0] > 40) {
        while (Math.abs(sensorColor.red() - sensorColor.blue()) < 5) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            sleep(10);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Place marker
        marker.setPosition(1);
        sleep(700);
        marker.setPosition(0);


        if (testMode) {
            while (!gamepad1.y && opModeIsActive()) {
                sleep(10);
            }
        }

        //Drive Toward the Crater

        gyroDrive(0.5,-67,10);

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
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = angles.firstAngle;
            while (headingAngle <= (targetAngle - threshold)  || headingAngle >= (targetAngle + threshold)) {
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

    public void pullUp(int position, double speed, double timeout) {
        runtime.reset();

        double startPosition = pullUp.getCurrentPosition();

        if(position > 0) {

            pullUp.setPower(speed);

            while ((pullUp.getCurrentPosition() < (position + startPosition)) && (runtime.seconds() < timeout)) {

                telemetry.addData("encoder value pullUp", pullUp.getCurrentPosition());
                telemetry.update();

            }
        }

        else if(position < 0) {

            pullUp.setPower(-speed);

            while (pullUp.getCurrentPosition() > (position + startPosition)&& (runtime.seconds() < timeout)) {

                telemetry.addData("encoder value arm", pullUp.getCurrentPosition());
                telemetry.update();
            }
        }

        pullUp.setPower(0);
    }

    public boolean checkGold() {
        boolean seesBlock = false;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }

//        /** Activate Tensor Flow Object Detection. */
//        if (tfod != null) {
//            tfod.activate();
//        }
//
//        sleep(1000);

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() > 0) {
                    if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                        seesBlock = true;
                    } else {
                        seesBlock = false;
                    }
                }
                telemetry.update();
            }
        }

//        if (tfod != null) {
//            tfod.shutdown();
//        }
        telemetry.addData("boolean value: ", seesBlock);
        telemetry.update();
        return seesBlock;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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

