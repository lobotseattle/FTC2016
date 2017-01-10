package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="DCMotor 2Wheel Drive Encoder Diff", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_2Wheel_Encoder_Diff extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    double MotorPower = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double PI = 3.1415;
        final double diameter = 4;
        final double DegreesPerEncoderTurn = 0.25;

        double circumference = diameter * PI;
        double distance = (2 * 14 * PI) / 4; //13.94*2;

        double rotationsNeededToReachTargetDistance = distance / circumference;
        double DegreesPerRotation = 360;

        //double DegreesForDistance = DegreesPerRotation * rotations;
        double encoderCountsPerRotation = 2880; //360*4 * 2
        double encoderTurnsToMakeLeft = encoderCountsPerRotation * rotationsNeededToReachTargetDistance;
        double encoderTurnsToMakeRight = 0;

        Thread driverThread = new DriverThread(encoderTurnsToMakeLeft, encoderTurnsToMakeRight);

        double rightRemaining = encoderTurnsToMakeRight - rightMotor.getCurrentPosition();
        double leftRemaining = encoderTurnsToMakeLeft - leftMotor.getCurrentPosition();

        double error = leftRemaining - rightRemaining;

        leftMotor.setPower(MotorPower - error / 2);
        rightMotor.setPower(MotorPower + error / 2);

        telemetry.addData("Status", "distance:" + distance);
        telemetry.update();
        telemetry.addData("Status", "encoderTurnsToMakeLeft:" + encoderTurnsToMakeLeft + "; encoderTurnsToMakeRight:" + encoderTurnsToMakeRight);
        telemetry.update();

        waitForStart();
        driverThread.start();


        while (opModeIsActive()) {

            try {
                int currentPositionLeft = leftMotor.getCurrentPosition();
                int currentPositionRight = rightMotor.getCurrentPosition();

                telemetry.addData("Status", "encoderTurnsToMake: " + encoderTurnsToMakeLeft + ", " + encoderTurnsToMakeRight + "; currentPosition: " + currentPositionLeft + ", " + currentPositionRight);
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("Exception", e.getMessage());
                telemetry.update();
            }

            driverThread.interrupt();
        }
    }

    private class DriverThread extends Thread {

        double encoderTurnsToMakeLeft;
        double encoderTurnsToMakeRight;

        public DriverThread(double encoderTurnsToMakeLeft, double encoderTurnsToMakeRight) {
            this.setName("DriverThread");
            encoderTurnsToMakeLeft = encoderTurnsToMakeRight;
            encoderTurnsToMakeRight = encoderTurnsToMakeRight;
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {

                    int currentPositionLeft = leftMotor.getCurrentPosition();
                    int currentPositionRight = rightMotor.getCurrentPosition();

                    if (currentPositionLeft >= encoderTurnsToMakeLeft && currentPositionRight >= encoderTurnsToMakeRight) {
                        //telemetry.addData("Status","Reached target distance. Breaking_Loop");
                        //telemetry.update();
                        break;
                    } else if (currentPositionLeft < encoderTurnsToMakeLeft && currentPositionRight >= encoderTurnsToMakeRight) {
                        leftMotor.setPower(MotorPower);
                        //telemetry.addData("Status","Set power to left motor only");
                        //telemetry.update();

                    } else if (currentPositionLeft >= encoderTurnsToMakeLeft && currentPositionRight < encoderTurnsToMakeRight) {
                        //telemetry.addData("Status","Set power to right motor only");
                        //telemetry.update();
                        rightMotor.setPower(MotorPower);
                    } else {
                        //telemetry.addData("Status","Set power to both motors");
                        //telemetry.update();
                        leftMotor.setPower(MotorPower);
                        rightMotor.setPower(MotorPower);
                    }

                    idle();
                }
                }
                catch(Exception e)
                {
                    telemetry.addData("Exception in driver thread", e.getMessage());
                    telemetry.update();
                }

            }
        }
}