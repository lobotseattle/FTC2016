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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        driverThread.start();


        //runtime.reset();


        // run until the end of the match (driver presses STOP)
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