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

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

//@TeleOp(name="DCMotor omnibot gamepad dpad touch", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_omnibot_dpad_touch extends OpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;
    DcMotor beaconMotor = null;
    // TouchSensor touchSensor = null;
    double motorSpeed = 0.3;
    // double beaconMotorSpeed = 0.1;

    // double spindleMotorPower = 0.5;
    // DcMotor shooterMotor = null;
    // double shooterMotorPower = 0.1;

    @Override
    public void init() {
        motorA = hardwareMap.dcMotor.get("motor_a");
        motorB = hardwareMap.dcMotor.get("motor_b");
        motorC = hardwareMap.dcMotor.get("motor_c");
        motorD = hardwareMap.dcMotor.get("motor_d");
        // beaconMotor = hardwareMap.dcMotor.get("beacon_motor");
        // beaconMotor = hardwareMap.dcMotor.get("motor_d");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorA.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        motorB.setDirection(DcMotor.Direction.REVERSE);
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */


        // boolean oldRBPressed = false;
        // boolean newRBPressed = false;
        // boolean isSpindleMotorOn = false;

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        // run until the end of the match (driver presses STOP)


                // double angle = getGamepadAngle(gamepad1.right_stick_x, gamepad1.right_stick_y);
                String direction = getDirectionFromDpad();
                moveToDirection(direction);
                // beaconMotorStatus();
    }

  /*  public void beaconMotorStatus ()
    {
        if (gamepad2.right_bumper)
        {
            beaconMotor.setPower(beaconMotorSpeed);
        }
        if (touchSensor.isPressed())
        {
            beaconMotor.setPower(0);
        }

    }
    */

    public String getDirectionFromDpad ()
    {
        if (gamepad1.dpad_up && gamepad1.dpad_left)
        {
            return "NorthWest";
        }
        if (gamepad1.dpad_left && gamepad1.dpad_down)
        {
            return "SouthWest";
        }
        if (gamepad1.dpad_down && gamepad1.dpad_right)
        {
            return "SouthEast";
        }
        if (gamepad1.dpad_up && gamepad1.dpad_right)
        {
            return "NorthEast";
        }

        if (gamepad1.dpad_up)
        {
            return "North";
        }

        if (gamepad1.dpad_left)
        {
            return "West";
        }
        if (gamepad1.dpad_down)
        {
            return "South";
        }
        if (gamepad1.dpad_right)
        {
            return "East";
        }
        if (gamepad1.y)
        {
            return "Clockwise";
        }
        if (gamepad1.x)
        {
            return "Counter Clockwise";
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
        if (direction == "Clockwise")
        {
            motorA.setPower(motorSpeed);
            motorB.setPower(motorSpeed);
            motorC.setPower(motorSpeed);
            motorD.setPower(motorSpeed);
        }
        if (direction == "Counter Clockwise")
        {
            motorA.setPower(-motorSpeed);
            motorB.setPower(-motorSpeed);
            motorC.setPower(-motorSpeed);
            motorD.setPower(-motorSpeed);
        }
    }

    public void moveToDirection (String direction, double distance) {

    }

    public void moveMotor(String motorName, double motorSpeed, double distance ) {
        while(distance >= 0 ) {
            if(motorName.equalsIgnoreCase("motorA")) {
                motorA.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("motorB")) {
                motorB.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("motorC")) {
                motorC.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("motorD")) {
                motorD.setPower(motorSpeed);
            }

        }

    }

}








