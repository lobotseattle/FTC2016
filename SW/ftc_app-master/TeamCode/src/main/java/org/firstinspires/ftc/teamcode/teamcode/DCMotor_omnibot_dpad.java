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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 Implements 4WD robot navigtion using omni wheels, positioned 90 degrees to each other.
 Implements rack and gear based ball throw system, with a limit switch simulated by push button sensor
 This is for tele op mode.
 Uses 6 motors, 1 push button sensor.
 */

@TeleOp(name="DCMotor omnibot gamepad dpad", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_omnibot_dpad extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // The 4 wheels that enable navigation
    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;

    // set the speed for all the wheels
    double motorSpeed = 0.1; // this is the motor speed for the launching motor/ motor E

    // this is the gear that controls the ball throw system
    DcMotor motorE = null;
    double gearMotorSpeed = 0.1;

    // this touch sensor controls ball throw system
    TouchSensor touch_sensor = null;
    boolean ballreleased = true;
    boolean ballpullstarted = false; // variable for if the ball
    boolean triggerIsPressed = false; // variable for if the right trigger was pressed
    boolean sensorIsPressed = false; // variable for if the touch sensor is pressed

    public void initialize()
    {
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorA = hardwareMap.dcMotor.get("motor_a");
        motorB = hardwareMap.dcMotor.get("motor_b");
        motorC = hardwareMap.dcMotor.get("motor_c");
        motorD = hardwareMap.dcMotor.get("motor_d");
        motorE = hardwareMap.dcMotor.get("motor_e");
        touch_sensor = hardwareMap.touchSensor.get("touch_sensor");

        // eg: Set the driv
        // e motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorA.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        motorB.setDirection(DcMotor.Direction.REVERSE);
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.REVERSE);
        motorE.setDirection(DcMotor.Direction.REVERSE);

        ballreleased = true;
        ballpullstarted = false; // variable for if the ball
        triggerIsPressed = false; // variable for if the right trigger was pressed
        sensorIsPressed = false; // variable for if the touch sensor is pressed
    }

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // gear motor start and stop
            if (gamepad2.right_trigger > 0.0)
            {
                triggerIsPressed = true;
            }

            if (ballpullstarted == false && triggerIsPressed == true)
            {
                ballpullstarted = true;
                triggerIsPressed = false;
                motorE.setPower(gearMotorSpeed);
            }

            if (touch_sensor.isPressed())
            {
                sensorIsPressed = true;
            }

            if (ballpullstarted == true && sensorIsPressed == true)
            {
                telemetry.addData("Touch Sensor: ", "Pressed");
                telemetry.update();
                ballreleased = true;
                ballpullstarted = false;
                sensorIsPressed = false;
                motorE.setPower(0);
            }

            // robot navigation
            String direction = getDirectionFromDpad();
            moveToDirection (direction);
        }
    }

    /*
     This method gets the direction from the dpad
     Eight directions: North, South, East, West, North [ East & West], South [East & West]
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

    /*
    This method moves the robot in the specified direction.
    The net direction of the movement is FNet on all the 4 motors
     */

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
            motorA.setPower(0);
            motorB.setPower(-motorSpeed);
            motorC.setPower(0);
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
            motorB.setPower(0);
            motorC.setPower(motorSpeed);
            motorD.setPower(0);
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
            motorA.setPower(0);
            motorB.setPower(motorSpeed);
            motorC.setPower(0);
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
            motorB.setPower(0);
            motorC.setPower(-motorSpeed);
            motorD.setPower(0);
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

    public void moveToDirection (String direction, double distance)
    {

    }

    /*
    Moves one motor. Used in diagnostics.
     */
    public void moveMotor(String motorName, double motorSpeed, double distance )
    {
        while(distance >= 0 )
        {
            if(motorName.equalsIgnoreCase("motorA")) {
                motorA.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("motorB")) {
                motorB.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("motorC")) {
                motorC.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("motorD")) {
                motorD.setPower(motorSpeed);
            }
            else if(motorName.equalsIgnoreCase("motorE")) {
                motorE.setPower(motorSpeed);
            }

        }
    }

}