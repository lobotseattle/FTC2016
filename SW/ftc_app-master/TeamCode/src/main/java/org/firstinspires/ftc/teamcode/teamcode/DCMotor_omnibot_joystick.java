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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

//@TeleOp(name="DCMotor omnibot gamepad joystick", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_omnibot_joystick extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;
    double motorSpeed = 0.1;
    // double spindleMotorPower = 0.5;
    // DcMotor shooterMotor = null;
    // double shooterMotorPower = 0.1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorA = hardwareMap.dcMotor.get("motor_a");
        motorB = hardwareMap.dcMotor.get("motor_b");
        motorC = hardwareMap.dcMotor.get("motor_c");
        motorD = hardwareMap.dcMotor.get("motor_d");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorA.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        motorB.setDirection(DcMotor.Direction.REVERSE);
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.REVERSE);

        // boolean oldRBPressed = false;
        // boolean newRBPressed = false;
        // boolean isSpindleMotorOn = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0)
            {
                motorA.setPower(0);
                motorB.setPower(0);
                motorC.setPower(0);
                motorD.setPower(0);
            }
            else {
                double angle = getGamepadAngle(gamepad1.right_stick_x, gamepad1.right_stick_y);
                String direction = getDirectionFromAngle(angle);
                moveToDirection(direction);
            }
        }
    }


    // x is adjacent length
    // y is opposite length
    public double getGamepadAngle(double x, double y)
    {
        double tanValue = 0.5;
        double angle = 45;

        if(x == 0 )
        {
            if(y > 0)
            {
                angle = 90;
            }
            else
            {
                angle = 270;
            }
        }
        else {
            tanValue = y / x;
            double radians = Math.atan(tanValue);
            angle = (radians * 180) / Math.PI;
        }
// todo find the correct fn
        telemetry.addData("The angle is set to ",angle + "(x,y): " + x + "," + y);
        telemetry.update();
        return angle;


    }
    public String getDirectionFromAngle (double angle)
    {
        if (angle >= 67.5 && angle < 112.5)
        {
            return "North";
        }
        if (angle >= 112.5 && angle < 157.5)
        {
            return "NorthWest";
        }
        if (angle >= 157.5 && angle < 202.5)
        {
            return "West";
        }
        if (angle >= 202.5 && angle < 247.5)
        {
            return "SouthWest";
        }
        if (angle >= 247.5 && angle < 292.5)
        {
            return "South";
        }
        if (angle >= 292.5 && angle < 337.5)
        {
            return "SouthEast";
        }
        if (angle >= 337.5 && angle < 22.5)
        {
            return "East";
        }
        if (angle >= 22.5 && angle < 67.5)
        {
            return "NorthEast";
        }
        else return "noDirection";
    }
    public void moveToDirection (String direction)
    {
        if (direction == "North")
        {
            motorA.setPower(motorSpeed);
            motorB.setPower(-motorSpeed);
            motorC.setPower(-motorSpeed);
            motorD.setPower(motorSpeed);
        }
        if (direction == "NorthWest")
        {
            motorA.setPower(motorSpeed);
            motorB.setPower(-motorSpeed);
            motorC.setPower(motorSpeed);
            motorD.setPower(motorSpeed);
        }
        if (direction == "West")
        {
            motorA.setPower(-motorSpeed);
            motorB.setPower(-motorSpeed);
            motorC.setPower(motorSpeed);
            motorD.setPower(motorSpeed);
        }
        if (direction == "SouthWest")
        {
            motorA.setPower(-motorSpeed);
            motorB.setPower(motorSpeed);
            motorC.setPower(motorSpeed);
            motorD.setPower(motorSpeed);
        }
        if (direction == "South")
        {
            motorA.setPower(-motorSpeed);
            motorB.setPower(motorSpeed);
            motorC.setPower(motorSpeed);
            motorD.setPower(-motorSpeed);
        }
        if (direction == "SouthEast")
        {
            motorA.setPower(-motorSpeed);
            motorB.setPower(motorSpeed);
            motorC.setPower(-motorSpeed);
            motorD.setPower(-motorSpeed);
        }
        if (direction == "East")
        {
            motorA.setPower(motorSpeed);
            motorB.setPower(motorSpeed);
            motorC.setPower(-motorSpeed);
            motorD.setPower(-motorSpeed);
        }
        if (direction == "NorthEast")
        {
            motorA.setPower(motorSpeed);
            motorB.setPower(-motorSpeed);
            motorC.setPower(-motorSpeed);
            motorD.setPower(-motorSpeed);
        }
        if (direction == "noDirection")
        {
            motorA.setPower(0);
            motorB.setPower(0);
            motorC.setPower(0);
            motorD.setPower(0);
        }
    }

    public void moveToDirection (String direction, double distance) {

    }

    public void moveMotor(String motorName, double motorSpeed, double distance ) {
        while(distance >= 0 ) {
            if(motorName.equalsIgnoreCase("frontLeft")) {
                motorA.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("frontRight")) {
                motorB.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("backRight")) {
                motorC.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("backLeft")) {
                motorD.setPower(motorSpeed);
            }

        }

    }

}








