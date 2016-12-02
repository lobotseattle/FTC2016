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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@TeleOp(name="DCMotor 2Wheel Drive Reverse Spindle", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_2WheelDrive_ReverseSpindle extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor leftSpindle = null;
    double spindleMotorPower = 0.5;
    DcMotor shooterMotor = null;
    double shooterMotorPower = 0.1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftSpindle = hardwareMap.dcMotor.get("left_spindle");
        shooterMotor = hardwareMap.dcMotor.get("shooter_motor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD ); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean oldRBPressed = false;
        boolean newRBPressed = false;
        boolean isSpindleMotorOn = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Inside loop");
            telemetry.update();

            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(-gamepad1.right_stick_y);

            if(gamepad2.b) {
                spindleMotorPower = incrementMotorPower(spindleMotorPower, "spindleMotor");
            }

            if(gamepad2.x) {
                shooterMotorPower = incrementMotorPower(shooterMotorPower, "shooterMotor");
            }
            if(gamepad2.left_trigger > 0)
            {
                shooterMotor.setPower(shooterMotorPower);
            }
            else
            {
                shooterMotor.setPower(0);
            }

            while(gamepad2.right_trigger > 0)
            {
                if(gamepad2.b) {
                    spindleMotorPower = incrementMotorPower(spindleMotorPower, "spindleMotor");
                }

                if(gamepad2.x) {
                    shooterMotorPower = incrementMotorPower(shooterMotorPower, "shooterMotor");
                }

                // telemetry.addData("Status", "spindleMotorPower: " + spindleMotorPower);
                // telemetry.update();

                // telemetry.addData("Status", "gamepad2.right_bumper: " + gamepad2.right_bumper);
                // telemetry.update();

                isSpindleMotorOn = !isSpindleMotorOn;
            }
            if (gamepad2.right_bumper)
            {
                leftSpindle.setPower(-0.3);
            }
            else
            {
                leftSpindle.setPower(0);
            }

            if (isSpindleMotorOn) {
                leftSpindle.setPower(spindleMotorPower);
            } else {
                leftSpindle.setPower(0);
            }
        }
    }

    public double incrementMotorPower(double oldValue, String motorName)
    {
        double newValue;

        if(oldValue >= 1.0)
        {
            newValue = 0;
        }
        else
        {
            newValue = oldValue + 0.1;
        }

        telemetry.addData("Status", motorName + ": " + newValue);
        telemetry.update();

        sleep(1000);

        return newValue;
    }

    boolean GetIsChanged(boolean oldValue, boolean newValue)
    {
        if(oldValue == newValue)
        {
            return false;
        }

        return true;
    }
}


