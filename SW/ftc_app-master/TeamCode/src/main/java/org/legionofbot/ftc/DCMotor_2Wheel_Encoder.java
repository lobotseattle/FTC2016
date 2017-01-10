
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="DCMotor 2Wheel Drive Encoder", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_2Wheel_Encoder extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

      @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor  = hardwareMap.dcMotor.get("left_drive");
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
          final double MotorPower = 0.4;

          double circumference = diameter * PI;
          double distance = (2 * 14 * PI)/4; //13.94*2;

          double rotationsNeededToReachTargetDistance = distance / circumference;
          double DegreesPerRotation = 360;

          //double DegreesForDistance = DegreesPerRotation * rotations;
          double encoderCountsPerRotation = 2880; //360*4 * 2
          double encoderTurnsToMakeLeft = encoderCountsPerRotation * rotationsNeededToReachTargetDistance;
          double encoderTurnsToMakeRight = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
          telemetry.addData("Status","distance:" + distance);
          telemetry.update();
          telemetry.addData("Status","encoderTurnsToMakeLeft:" + encoderTurnsToMakeLeft + "; encoderTurnsToMakeRight:" + encoderTurnsToMakeRight);
          telemetry.update();

        //runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int currentPositionLeft = leftMotor.getCurrentPosition();
            int currentPositionRight = rightMotor.getCurrentPosition();

            telemetry.addData("Status","encoderTurnsToMakeLeft: " + encoderTurnsToMakeLeft + "encoderTurnsToMakeRight: " + encoderTurnsToMakeRight + "; currentPosition: " + currentPositionLeft + "; " + currentPositionRight);
            telemetry.update();

            if(currentPositionLeft >= encoderTurnsToMakeLeft && currentPositionRight >= encoderTurnsToMakeRight) {
                //telemetry.addData("Status","Reached target distance. Breaking_Loop");
                //telemetry.update();
                break;
            }

            else if(currentPositionLeft < encoderTurnsToMakeLeft && currentPositionRight >= encoderTurnsToMakeRight) {
                leftMotor.setPower(MotorPower);
                //telemetry.addData("Status","Set power to left motor only");
                //telemetry.update();

            }
            else if(currentPositionLeft >= encoderTurnsToMakeLeft && currentPositionRight < encoderTurnsToMakeRight) {
                //telemetry.addData("Status","Set power to right motor only");
                //telemetry.update();
                rightMotor.setPower(MotorPower);
            }
            else {
                //telemetry.addData("Status","Set power to both motors");
                //telemetry.update();
                leftMotor.setPower(MotorPower);
                rightMotor.setPower(MotorPower);
            }
        }
    }
}
