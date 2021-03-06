package org.legionofbot.ftc;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="DCMotor 2Wheel Drive Encoder2 4Wheels", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_2Wheel_Encoder2_4Wheels extends LinearOpMode {

    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;
    DcMotor motorE = null;
    ModernRoboticsI2cGyro gyro_sensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    String log = "";

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; //4.0 ;     // For figuring circumference
    static final double PI = 3.1415;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 1.0;
    static final double radiusOfRobotRotationPath = 14;
    private double distanceFor360Degrees = 2 * PI * radiusOfRobotRotationPath;
    static final double positionErrorBuffer = COUNTS_PER_INCH / 10;
    static final double vectorFactorMultiplier = Math.sqrt(2);

    static final double TIMEOUTINSECONDS = 30;

    double motorSpeed = 0.5;

    @Override
    public void runOpMode() {

        motorA = hardwareMap.dcMotor.get("motor_a");
        motorB = hardwareMap.dcMotor.get("motor_b");
        motorC = hardwareMap.dcMotor.get("motor_c");
        motorD = hardwareMap.dcMotor.get("motor_d");
        motorE = hardwareMap.dcMotor.get("motor_e");

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean ballreleased = true;
        boolean ballpullstarted = false;
        boolean triggerIsPressed = false;
        boolean sensorIsPressed = false;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData
        (
            "Path0", "Starting at %7d :%7d : %7d : %7d : %7d",
            motorA.getCurrentPosition(),
            motorB.getCurrentPosition(),
            motorC.getCurrentPosition(),
            motorD.getCurrentPosition(),
            motorE.getCurrentPosition()
        );

        telemetry.update();

        CalibrateGyro();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Started","Auto mode");
        telemetry.update();


        while (opModeIsActive())
        {

        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  12,  12, 25);  // S1: Forward 47 Inches with 5 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void CalibrateGyro()
    {
        gyro_sensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro_sensor");

        telemetry.addData("Status", "Starting calibration");    //
        telemetry.update();

        gyro_sensor.calibrate();
        while(gyro_sensor.isCalibrating())
        {
            telemetry.addData("Status", "Calibrating");
            telemetry.update();
        }

        telemetry.addData("Status", "Finished calibrating");
        telemetry.update();
    }

    public void TestGyro()
    {
      // telemetry.addData("Status", "GYRO RotationFraction, X,Y,Z:" +  "; " + "; " +  gyro_sensor.getRotationFraction() + "; " +   gyro_sensor.rawX() + "; " + gyro_sensor.rawY() + "; " + gyro_sensor.rawZ());    //
        this.gyro_sensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        int heading = this.gyro_sensor.getHeading();
       telemetry.addData("Status", heading);
       telemetry.update();
        sleep(3000);
    }

    public void startActions()
    {
        telemetry.addData("StartActions", "Base class Called");
        telemetry.update();
        /*encoderDrive(DRIVE_SPEED, 12, 12 , 25);  // Go forward straight 12 inches
        encoderDrive(DRIVE_SPEED, distanceFor360Degrees/8, -distanceFor360Degrees/8 , 25);  // Go 90 degrees left
        encoderDrive(DRIVE_SPEED, -distanceFor360Degrees/8, distanceFor360Degrees/8, 25);  // Go 90 degrees right
        encoderDrive(DRIVE_SPEED, -12, -12 , 25);  // Go reverse straight 12 inches*/
    }

    public void moveToDirection(String direction, double motorSpeed, double distance)
    {
        double newdistance = distance / vectorFactorMultiplier;

        if (direction == "North") {
            moveMotor(newdistance, -newdistance, -newdistance, newdistance, motorSpeed);
        }
        if (direction == "NorthEast") {
            moveMotor(0, -distance, 0, distance, motorSpeed);
        }
        if (direction == "East") {
            moveMotor(-newdistance, -newdistance, newdistance, newdistance, motorSpeed);
        }
        if (direction == "SouthEast") {
            moveMotor(-distance, 0, distance, 0, motorSpeed);
        }
        if (direction == "South") {
            moveMotor(-newdistance, newdistance, newdistance, -newdistance, motorSpeed);
        }
        if (direction == "SouthWest") {
            moveMotor(0, distance, 0, -distance, motorSpeed);
        }
        if (direction == "West") {
            moveMotor(newdistance, newdistance, -newdistance, -newdistance, motorSpeed);
        }
        if (direction == "NorthWest") {
            moveMotor(distance, 0, -distance, 0, motorSpeed);
        }
        if (direction == "noDirection") {
            moveMotor(0, 0, 0, 0, motorSpeed);
        }
        if (direction == "CounterClockWise") {
            moveMotor(distance,distance,distance,distance, motorSpeed);
        }
        if (direction == "Clockwise") {
            moveMotor(-distance,-distance,-distance,-distance,motorSpeed);
        }
    }


    public void moveMotor(String motorName, double motorSpeed, double distance ) {
        while (distance >= 0) {
            if (motorName.equalsIgnoreCase("frontLeft")) {
                motorA.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("frontRight")) {
                motorB.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("backRight")) {
                motorC.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("backLeft")) {
                motorD.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("motorE")) {
                motorE.setPower(motorSpeed);
            }
        }
    }

    public void moveMotor(double motorADistance, double motorBDistance, double motorCDistance, double motorDDistance, double speed) {
        encoderDrive(motorADistance, motorBDistance, motorCDistance, motorDDistance, speed, 30000);
    }

    public void encoderDriveInches(
            double motorAInches,
            double motorBInches,
            double motorCInches,
            double motorDInches,
            double motorSpeed,
            double timeoutS){

    }

    public void encoderDrive(
            double motorADistance, double motorBDistance, double motorCDistance, double motorDDistance, double speed, double timeoutS) {

        int newTargetA = 0;
        int newTargetB = 0;
        int newTargetC = 0;
        int newTargetD = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            if(motorADistance != 0)
            {
                // Determine new target position, and pass to motor controller
                newTargetA = motorA.getCurrentPosition() + (int) (motorADistance * COUNTS_PER_INCH);
                motorA.setTargetPosition(newTargetA);
                // Turn On RUN_TO_POSITION
                motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(motorBDistance != 0)
            {
                newTargetB = motorB.getCurrentPosition() + (int) (motorBDistance * COUNTS_PER_INCH);
                motorB.setTargetPosition(newTargetB);
                motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(motorCDistance != 0)
            {
                newTargetC = motorC.getCurrentPosition() + (int) (motorCDistance * COUNTS_PER_INCH);
                motorC.setTargetPosition(newTargetC);
                motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(motorDDistance != 0)
            {
                newTargetD = motorD.getCurrentPosition() + (int) (motorDDistance * COUNTS_PER_INCH);
                motorD.setTargetPosition(newTargetD);
                motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            if(motorADistance != 0)
            {
                motorA.setPower(Math.abs(speed));
            }
            if(motorBDistance != 0)
            {
                motorB.setPower(Math.abs(speed));
            }
            if(motorCDistance != 0)
            {
                motorC.setPower(Math.abs(speed));
            }
            if(motorDDistance != 0)
            {
                motorD.setPower(Math.abs(speed));
            }


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    //(motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy())
                    (isMotorBusy(motorA) || isMotorBusy(motorB) || isMotorBusy(motorC) || isMotorBusy(motorD))
                    )
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "RUNNING - to %7d : %7d : %7d : %7d", newTargetA, newTargetB, newTargetC, newTargetD);
                telemetry.addData("Path2",  "RUNNING - at %7d : %7d : %7d : %7d",
                        (motorADistance ==0) ? 0: motorA.getCurrentPosition(),
                        (motorBDistance ==0) ? 0: motorB.getCurrentPosition(),
                        (motorCDistance ==0) ? 0: motorC.getCurrentPosition(),
                        (motorDDistance ==0) ? 0: motorD.getCurrentPosition());

              telemetry.update();
            }

            // Stop all motion;
            motorA.setPower(0);
            motorB.setPower(0);
            motorC.setPower(0);
            motorD.setPower(0);

            // Turn off RUN_TO_POSITION
            motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private boolean isMotorBusy (DcMotor motor)
    {
        boolean returnValue = false;
        if (motor != null )
        {
            int currentPosition = motor.getCurrentPosition();
            int targetPosition = motor.getTargetPosition();
            int diff = currentPosition - targetPosition;

            if (Math.abs(diff) > positionErrorBuffer ) returnValue = true;
        }
        return returnValue;
    }
    public void moveMotorDistance(DcMotor motor, double distanceInInches, double motorSpeed)
    {
        if (motor != null)
        {
            int targetPosition = motor.getCurrentPosition() + (int) (distanceInInches * COUNTS_PER_INCH);
            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(motorSpeed));
            while (opModeIsActive() &&
                    (runtime.seconds() < 30000) &&
                    (motor.isBusy() )
                    )
            {
                telemetry.addData("Motor Test",  "RUNNING - to %7d", targetPosition);
                telemetry.update();
            }
        }
    }

    public void rotateRobot (int degrees)
    {
        double turningSpeed = 0.1;
        // set the mode of te gyro
        this.gyro_sensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        // get the current hesding value
        int currentHeading = this.gyro_sensor.getHeading();

        // compute the target. Take care of the 360 degree fold
        // the gyro values go from 0-359
        // the best way to calculate is to use the modulus operator to calculate the remainder
        int targetHeading = (currentHeading + degrees + 360 ) % 360;
        // calculate angular distance both in clockwise and in counter clockwise directions
        int CWDistance = targetHeading - currentHeading;
        int CCWDistance = 360 - CWDistance;
        int deltaDistance = CCWDistance-CWDistance;

        // defalt is clockwise direction
        boolean clockwiseMotion = true;

        if (deltaDistance < 0 ) clockwiseMotion = false ;

        telemetry.addData("Current Angle", currentHeading);
        telemetry.addData("Target Angle", targetHeading);
        telemetry.update();
        sleep(3000);

        if (!clockwiseMotion)
        {
            // move counter clockwise to the desired angle
            motorA.setPower(turningSpeed);
            motorB.setPower(turningSpeed);
            motorC.setPower(turningSpeed);
            motorD.setPower(turningSpeed);
        }
        else
        {
            // move clockwise to the desired angle
            motorA.setPower(-turningSpeed);
            motorB.setPower(-turningSpeed);
            motorC.setPower(-turningSpeed);
            motorD.setPower(-turningSpeed);
        }

        while (opModeIsActive() &&
                (runtime.seconds() < 30000)   )
        {
            int currentReading = this.gyro_sensor.getHeading();
            telemetry.addData("Target Angle", targetHeading);
            telemetry.addData("Current Angle", currentReading);
            String direction = "Clockwise"; if (clockwiseMotion == false) { direction="Counter Clockwise"; }
            telemetry.addData("Direction", direction);
            telemetry.update();
            if (Math.abs(currentReading-targetHeading) <= 1)
            {
                motorA.setPower(0);
                motorB.setPower(0);
                motorC.setPower(0);
                motorD.setPower(0);
                break;
            }
        }
    }
}
