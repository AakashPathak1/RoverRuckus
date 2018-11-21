package org.firstinspires.ftc.robotcontroller.internal;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@TeleOp(name="mechanumDriveOnly", group="8872")
@Disabled
public class MechanumDriveOnly extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;





    static final double APPROACH_SPEED = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX =  gamepad1.right_stick_x;
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

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);





        }

    }


}

