package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="DCMotor 2Wheel Drive Encoder2 BothWheelsTurn2", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_2Wheel_Encoder2_BothWheelsTurn extends LinearOpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor leftSpindle = null;
    DcMotor shooterMotor = null;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0; //4.0 ;     // For figuring circumference
    static final double     PI    = 3.1415 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 1.0;
    static final double radiusOfRobotRotationPath = 14;
    private double distanceFor360Degrees = 2 * PI * radiusOfRobotRotationPath;

    static final double TIMEOUTINSECONDS = 30;

    @Override
    public void runOpMode() {

        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftSpindle = hardwareMap.dcMotor.get("left_spindle");
        shooterMotor = hardwareMap.dcMotor.get("shooter_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  12,  12, 25);  // S1: Forward 47 Inches with 5 Sec timeout

        startActions();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void startActions()
    {
    }

    public void forward(double distance)
    {
        encoderDrive(DRIVE_SPEED, distance, distance, TIMEOUTINSECONDS);  // Go forward straight 12 inches
    }

    public void reverse(double distance)
    {
        encoderDrive(DRIVE_SPEED, -distance, -distance , TIMEOUTINSECONDS);  // Go forward straight 12 inches
    }

    public void turnLeft(double degrees)
    {
        double distanceForDegrees = calculateDistanceFromDegrees(degrees);
        encoderDrive(DRIVE_SPEED, distanceForDegrees/2, -distanceForDegrees/2 , 25);  // Go 90 degrees left
    }

    public void turnRight(double degrees)
    {
        double distanceForDegrees = calculateDistanceFromDegrees(degrees);
        encoderDrive(DRIVE_SPEED, -distanceForDegrees/2, distanceForDegrees/2, 25);
    }

    private double calculateDistanceFromDegrees(double degrees)
    {
        return distanceFor360Degrees / (360/degrees);
    }

    public void shooterOn(double motorPower)
    {
        shooterMotor.setPower(motorPower);
    }

    public void shooterOff()
    {
        shooterMotor.setPower(0);
    }

    public void spindleOn(double motorPower)
    {
        leftSpindle.setPower(motorPower);
    }

    public void spindleOff()
    {
        leftSpindle.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() || rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "RUNNING - to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "RUNNING - at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
