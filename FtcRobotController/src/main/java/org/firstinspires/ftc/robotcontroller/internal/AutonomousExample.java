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


//@Autonomous(name="RegionalsAutoBlueCorner", group="Pushbot")
public class AutonomousExample extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    VuforiaLocalizer vuforia;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    static final double     TURN_SPEED              = 0.6;     // Nominal half speed for better accuracy.
    static final double     MINIMUM_TURN_SPEED      = 0.3;
    static final double     DRIVE_SPEED             = 0.6;
    static final double APPROACH_SPEED = 0.2;
    static final double SPEED_FACTOR = 1.0;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor pullGlyphRight;
    DcMotor pullGlyphLeft;
    DcMotor liftMotor;
    DcMotor relicMotor;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    Servo jewelDropper;
    Servo jewelHitter;
    Servo relicUp;
    Servo relicOpen;
    Servo placeGlyph;


    @Override
    public void runOpMode() {

        double headingAngle;

        double JEWEL_ON_POS = 0.2;
        double JEWEL_OFF_POS = 0.8;
        double JEWEL_TURN_MIDDLE = 0.73;
        double JEWEL_TURN_FORWARD = 0.3;
        double JEWEL_TURN_BACK = 1.0;
        double relicUpOnPos = 0.0;
        double relicUpOffPos = 0.4;
        double relicOpenOnPos = 1.0;
        double relicOpenOffPos = 0.3;
        double placeGlyphUp = 0.0;
        double placeGlyphDown = 1.0;
        int vuMarkID = 0; //0 = UNKNOWN, LEFT = 1, CENTER = 2, RIGHT = 3

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vParameters.vuforiaLicenseKey = "AWpzsnX/////AAAAGR6hjv+pxEhwo5LMDdyVUy0xvKokFq98ViLeRFm2" +
                "rmOx1ejvN2xSgYjsteGUL/IBvrWr9TrClrhPm34DZeGnej3cYb0DqSrUUT2OT4jos5ZYpxwDP8uD14G7G8NPwe" +
                "Wve+P9Sv+BcvjDJ4NgReJqtfD+SpH59wBfneSylGdAmdpPghQPwAv5j5j4Y6xDHflbxHWEr/hVify/CSPzkmEhFGtIDXkRkjO" +
                "14hih6zGh+dRAsuUIlN+So55XJ0PR7eU3tAGotA9u7yTkFqE7f0OciD3Vgz4Klf7lzoTyz+SNWtbhXVqNcBOlGhtz6nucAz2xWgWoeF" +
                "LQiamHAeqbrWTvrv6afFCz7m9x25XVK8EpZPNV";

        vParameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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
        pullGlyphRight = hardwareMap.dcMotor.get("glyphRight");
        pullGlyphLeft = hardwareMap.dcMotor.get("glyphLeft");
        jewelDropper = hardwareMap.servo.get("jewelDropper");
        jewelHitter = hardwareMap.servo.get("jewelHitter");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        relicMotor = hardwareMap.dcMotor.get("relicMotor");
        placeGlyph = hardwareMap.servo.get("placeGlyph");
        relicUp = hardwareMap.servo.get("relicUp");
        relicOpen = hardwareMap.servo.get("relicOpen");
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");


        telemetry.update();

        relicUp.setPosition(relicUpOffPos);
        relicOpen.setPosition(relicOpenOffPos);

        while (!isStopRequested() && imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        pullGlyphRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullGlyphRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullGlyphLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        headingAngle = angles.firstAngle;


        jewelHitter.setPosition(JEWEL_TURN_MIDDLE);

        jewelDropper.setPosition(JEWEL_OFF_POS);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.update();

        relicTrackables.activate();

        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();


        while (!isStarted()) {
            headingAngle = angles.firstAngle;
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("headingAngle", headingAngle);
            telemetry.update();

        }
        //Begin Program

        //BEGIN PROGRAM

        if (vuMark == RelicRecoveryVuMark.LEFT){
            vuMarkID = 1;
        } else if (vuMark == RelicRecoveryVuMark.CENTER){
            vuMarkID = 2;
        } else if(vuMark == RelicRecoveryVuMark.RIGHT){
            vuMarkID = 3;
        }


        //TEMPORARY
        //vuMarkID = 1;

        for (double i = JEWEL_OFF_POS; i >= JEWEL_ON_POS;i-=0.005) {
            jewelDropper.setPosition(i);
            jewelHitter.setPosition(JEWEL_TURN_MIDDLE);
        }
        sleep(250);
        //Detect color
        int blueValue = sensorColor.blue();
        int redValue = sensorColor.red();

        telemetry.addData("Red", redValue);
        telemetry.addData("Blue", blueValue);

        telemetry.update();

        if (redValue > blueValue) { //if its red
            telemetry.addLine("ITS RED");
            jewelHitter.setPosition(JEWEL_TURN_BACK);
        } else if (redValue < blueValue) { //if its blue
            telemetry.addLine("ITS BLUE");
            jewelHitter.setPosition(JEWEL_TURN_FORWARD);
        }
        sleep(200);

        jewelDropper.setPosition(0.6);
        jewelHitter.setPosition(JEWEL_TURN_MIDDLE);
        jewelDropper.setPosition(JEWEL_OFF_POS);
        telemetry.update();

        sleep(50);

        gyroDrive(DRIVE_SPEED, -24, 7);
        sleep(100);

        if (vuMarkID == 1){
            gyroTurn(TURN_SPEED, -5, 10, 5);
            gyroDrive(DRIVE_SPEED, -3, -5,7);
        } else if (vuMarkID == 2){
            gyroSideDrive(DRIVE_SPEED,4.5,0);
            gyroDrive(DRIVE_SPEED,2,4);
            gyroTurn(TURN_SPEED, -24, 15, 5);
            gyroDrive(DRIVE_SPEED, -4, -24,7);
        } else if(vuMarkID == 3){
            gyroDrive(DRIVE_SPEED,3,7);
            gyroTurn(TURN_SPEED, -90, 80, 5);
            sleep(400);
            gyroDrive(DRIVE_SPEED, -9, -90,7);
            sleep(400);
            gyroTurn(TURN_SPEED, -24, 10, 5);
            sleep(400);
            gyroDrive(DRIVE_SPEED, -5, -24
                    ,7);
            sleep(400);

        }

        pullGlyphLeft.setPower(-1);
        pullGlyphRight.setPower(-1);
        sleep(1000);

        gyroDrive(DRIVE_SPEED, 8, 7);

        pullGlyphLeft.setPower(0);
        pullGlyphRight.setPower(0);

        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyroTurn(TURN_SPEED,90,80,7);

        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyroTurn(TURN_SPEED,90,80,7);

        gyroDrive(DRIVE_SPEED,8,7);
        gyroDrive(DRIVE_SPEED,-5,7);


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
            leftRear.setPower(-speed);
            rightRear.setPower(speed);

            while (leftRear.getCurrentPosition() > (encoderCount + startPosition)) {
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
