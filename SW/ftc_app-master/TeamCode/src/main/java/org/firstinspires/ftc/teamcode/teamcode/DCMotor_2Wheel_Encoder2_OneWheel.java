/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="DCMotor 2Wheel Drive Encoder2 OneWheel", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_2Wheel_Encoder2_OneWheel extends LinearOpMode {

    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;

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

        motorA  = hardwareMap.dcMotor.get("motor_a");
        motorB = hardwareMap.dcMotor.get("motor_b");
        motorC = hardwareMap.dcMotor.get("motor_c");
        motorD = hardwareMap.dcMotor.get("motor_d");

        motorA.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorB.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorC.setDirection(DcMotor.Direction.FORWARD);
        motorD.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d : %7d : %7d",
                motorA.getCurrentPosition(),
                motorB.getCurrentPosition(),
        motorC.getCurrentPosition(),
        motorD.getCurrentPosition());
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
        /*encoderDrive(DRIVE_SPEED, 12, 12 , 25);  // Go forward straight 12 inches
        encoderDrive(DRIVE_SPEED, distanceFor360Degrees/8, -distanceFor360Degrees/8 , 25);  // Go 90 degrees left
        encoderDrive(DRIVE_SPEED, -distanceFor360Degrees/8, distanceFor360Degrees/8, 25);  // Go 90 degrees right
        encoderDrive(DRIVE_SPEED, -12, -12 , 25);  // Go reverse straight 12 inches*/


    }

  /*  public void forward(double distance)
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
        encoderDrive("motorA", DRIVE_SPEED, -distanceForDegrees/2, 25);
    }*/

    public void encoderDrive(
            double motorASpeed,
            double motorBSpeed,
            double motorCSpeed,
            double motorDSpeed,
            double inches,
            double timeoutS) {

        int newTargetA;
        int newTargetB;
        int newTargetC;
        int newTargetD;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if(motorASpeed != 0) {
                // Determine new target position, and pass to motor controller
                newTargetA = motorA.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                motorA.setTargetPosition(newTargetA);
                // Turn On RUN_TO_POSITION
                motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(motorBSpeed != 0) {
                newTargetB = motorB.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                motorB.setTargetPosition(newTargetB);
                motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(motorCSpeed != 0) {
                newTargetC = motorC.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                motorC.setTargetPosition(newTargetC);
                motorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(motorDSpeed != 0) {
                newTargetD = motorD.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                motorD.setTargetPosition(newTargetD);
                motorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            motorA.setPower(Math.abs(motorASpeed));
            motorB.setPower(Math.abs(motorBSpeed));
            motorC.setPower(Math.abs(motorCSpeed));
            motorD.setPower(Math.abs(motorDSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorA.isBusy() || motorB.isBusy() || motorC.isBusy() || motorD.isBusy())
                    ) {

                // Display it for the driver.
                telemetry.addData("Path1",  "RUNNING - to %7d : %7d : %7d : %7d", motorA, motorB, motorC, motorD);
                telemetry.addData("Path2",  "RUNNING - at %7d : %7d : %7d : %7d",
                        (motorASpeed ==0) ? 0: motorA.getCurrentPosition(),
                        (motorBSpeed ==0) ? 0: motorB.getCurrentPosition(),
                        (motorCSpeed ==0) ? 0: motorC.getCurrentPosition(),
                        (motorDSpeed ==0) ? 0: motorD.getCurrentPosition());

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
}
