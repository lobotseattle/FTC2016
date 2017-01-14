
package org.legionofbot.ftc;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

//@Autonomous(name="OpMode Robot", group="Linear Opmode")
public class OpMode_Robot extends OpMode {

    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;
    DcMotor shooterMotor = null;

    int newTargetA = 0;
    int newTargetB = 0;
    int newTargetC = 0;
    int newTargetD = 0;

    double motorADistance;
    double motorBDistance;
    double motorCDistance;
    double motorDDistance;
    double speed;

    double timeoutS;

    double sleepMilliseconds = 0;

    RobotActionModes robotActionMode;

    ArrayList<IRobotAction> lstActions = new ArrayList<IRobotAction>();;

    int currentActionIndex = 0;
    boolean targetActive = false;

    int targetHeading = 0;
    double turningSpeed = 0.1;
    boolean clockwiseMotion = true;

    ModernRoboticsI2cGyro gyro_sensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    String log = "";

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; //4.0 ;     // For figuring circumference
    static final double PI = 3.1415;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 1.0;
    static final double radiusOfRobotRotationPath = 14;
    private double distanceFor360Degrees = 2 * PI * radiusOfRobotRotationPath;
    static final double positionErrorBuffer = COUNTS_PER_INCH / 10;
   // static final double positionErrorBuffer = COUNTS_PER_INCH / 5;
    static final double vectorFactorMultiplier = Math.sqrt(2);

    static final double TIMEOUTINSECONDS = 30;

    double motorSpeed = 0.5;


    @Override
    public void init() {

        motorA = hardwareMap.dcMotor.get("motor_a");
        motorB = hardwareMap.dcMotor.get("motor_b");
        motorC = hardwareMap.dcMotor.get("motor_c");
        motorD = hardwareMap.dcMotor.get("motor_d");
        shooterMotor = hardwareMap.dcMotor.get("motor_shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //idle();

        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean ballreleased = true;
        boolean ballpullstarted = false;
        boolean triggerIsPressed = false;
        boolean sensorIsPressed = false;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData
                (
                        "Path0", "Starting at %7d :%7d : %7d : %7d",
                        motorA.getCurrentPosition(),
                        motorB.getCurrentPosition(),
                        motorC.getCurrentPosition(),
                        motorD.getCurrentPosition()
                );

        telemetry.update();

        CalibrateGyro();

        registerActions();

    }
        public void loop() {


        {

            startActions(telemetry);
        }

            telemetry.addData("Status", "currentActionIndex:" + currentActionIndex);
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
        this.gyro_sensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        int heading = this.gyro_sensor.getHeading();
       telemetry.addData("Status", heading);
       telemetry.update();

    }

    public void registerActions()
    {
    }

    public void startActions(Telemetry telemetry)
    {
        telemetry.addData("StartActions", "Base class Called");
        telemetry.update();
        }

    public void driveRobotInDirection(String direction, double motorSpeed, double distance)
    {
        telemetry.addData("Status", "GoStraight " + direction + ": " + distance);
        telemetry.update();

        double newdistance = distance / vectorFactorMultiplier;

        if (direction == "North") {
            driveMotors(newdistance, -newdistance, -newdistance, newdistance, motorSpeed);
        }
        if (direction == "NorthEast") {
            driveMotors(0, -distance, 0, distance, motorSpeed);
        }
        if (direction == "East") {
            driveMotors(-newdistance, -newdistance, newdistance, newdistance, motorSpeed);
        }
        if (direction == "SouthEast") {
            driveMotors(-distance, 0, distance, 0, motorSpeed);
        }
        if (direction == "South") {
            driveMotors(-newdistance, newdistance, newdistance, -newdistance, motorSpeed);
        }
        if (direction == "SouthWest") {
            driveMotors(0, distance, 0, -distance, motorSpeed);
        }
        if (direction == "West") {
            driveMotors(newdistance, newdistance, -newdistance, -newdistance, motorSpeed);
        }
        if (direction == "NorthWest") {
            driveMotors(distance, 0, -distance, 0, motorSpeed);
        }
        if (direction == "noDirection") {
            driveMotors(0, 0, 0, 0, motorSpeed);
        }
        if (direction == "CounterClockWise") {
            driveMotors(distance,distance,distance,distance, motorSpeed);
        }
        if (direction == "Clockwise") {
            driveMotors(-distance,-distance,-distance,-distance,motorSpeed);
        }
    }


    public void driveMotors(double motorADistance, double motorBDistance, double motorCDistance, double motorDDistance, double speed) {
        if(!targetActive) {
            setGoStraightTarget(motorADistance, motorBDistance, motorCDistance, motorDDistance, speed, 30000);
            targetActive = true;
        }
        else
        {
            moveStraightToTarget();
        }
    }

    public void encoderDriveInches(
            double motorAInches,
            double motorBInches,
            double motorCInches,
            double motorDInches,
            double motorSpeed,
            double timeoutS){

    }

    public void moveStraightToTarget()
    {
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
        if ((runtime.seconds() < timeoutS) &&
                //(motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy())
                (isMotorBusy(motorA) || isMotorBusy(motorB) || isMotorBusy(motorC) || isMotorBusy(motorD))
                )
        {

        }

        else {

            targetActive = false;

            telemetry.addData("Status", "Reached target");
            currentActionIndex++;

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


    public void setGoStraightTarget(double motorADistance, double motorBDistance, double motorCDistance, double motorDDistance, double speed, double timeoutS) {

        this.motorADistance = motorADistance;
        this.motorBDistance = motorBDistance;
        this.motorCDistance  = motorCDistance;
        this.motorDDistance = motorDDistance;
        this.speed = speed;
        this.timeoutS = timeoutS;

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
            if ((runtime.seconds() < 30000) && motor.isBusy())
            {
                telemetry.addData("Motor Test",  "RUNNING - to %7d", targetPosition);
                telemetry.update();
            }
        }
    }

    public void rotateRobot(double turnSpeed, int degrees)
    {
        if(!targetActive) {
            setRotationTarget(turnSpeed, degrees);
            targetActive = true;
        }
        else
        {
            rotateToTarget();
        }
    }

    public void setRotationTarget (double turnSpeed, int degrees) {
        turningSpeed = turnSpeed;
        // set the mode of te gyro
        this.gyro_sensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        // get the current hesding value
        int currentHeading = this.gyro_sensor.getHeading();

        // compute the target. Take care of the 360 degree fold
        // the gyro values go from 0-359
        // the best way to calculate is to use the modulus operator to calculate the remainder
        targetHeading = (currentHeading + degrees + 360 ) % 360;
        // calculate angular distance both in clockwise and in counter clockwise directions
        int CWDistance = targetHeading - currentHeading;
        int CCWDistance = 360 - CWDistance;
        int deltaDistance = CCWDistance-CWDistance;

        // defalt is clockwise direction
        clockwiseMotion = true;

        if (deltaDistance < 0 ) clockwiseMotion = false ;

        telemetry.addData("Current Angle", currentHeading);
        telemetry.addData("Target Angle", targetHeading);
        telemetry.update();
        //sleep(3000);
    }


    public void rotateToTarget ()
    {
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

        // if (runtime.seconds() < 300000)
        //{
            int currentReading = this.gyro_sensor.getHeading();
            telemetry.addData("Target Angle", targetHeading);
            telemetry.addData("Current Angle", currentReading);
            String direction = "Clockwise"; if (clockwiseMotion == false) { direction="Counter Clockwise"; }
            telemetry.addData("Direction", direction);
            telemetry.update();
            if (Math.abs(currentReading-targetHeading) <= 2)
            {
                targetActive = false;
                currentActionIndex++;
                motorA.setPower(0);
                motorB.setPower(0);
                motorC.setPower(0);
                motorD.setPower(0);
            }
        //}
    }

    public void sleepRobot(double milliSeconds, boolean turnOffshootMotorWhenDone)
    {
        if(!targetActive) {
            setSleepTarget(milliSeconds);
            targetActive = true;
        }
        else
        {
            sleepToTarget(turnOffshootMotorWhenDone);
        }
    }

    private void setSleepTarget(double milliSeconds)
    {
        runtime.reset();
        sleepMilliseconds = milliSeconds;
    }

    private void sleepToTarget(boolean turnOffshootMotorWhenDone)
    {
        if(runtime.milliseconds() >= sleepMilliseconds) {
            shooterMotor.setPower(0);
            targetActive = false;
            currentActionIndex++;
        }
    }
}
