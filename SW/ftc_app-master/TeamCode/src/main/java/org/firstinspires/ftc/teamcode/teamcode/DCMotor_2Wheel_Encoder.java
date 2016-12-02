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
