package org.firstinspires.ftc.robotcontroller.internal;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

/**
 * {@link } gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
//@Autonomous(name = "Gyro Turn Test", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class GyroTurnTest extends LinearOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    private ElapsedTime runtime = new ElapsedTime();

    static final double APPROACH_SPEED = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     TURN_SPEED              = 0.6;     // Nominal half speed for better accuracy.
    static final double     MINIMUM_TURN_SPEED      = 0.3;
    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.setMsTransmissionInterval(100);

        double headingAngle = angles.firstAngle;

        while (!isStarted()) {
            headingAngle = angles.firstAngle;
            telemetry.addData("headingAngle", headingAngle);
            telemetry.update();

        }
        gyroTurn(TURN_SPEED, -90, 10, 5);

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("Linear Acceleration", new Func<String>() {
                    @Override public String value() {
                        return imu.getLinearAcceleration().toString();
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
                })
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

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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
}
