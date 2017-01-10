
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    public void init()
    {
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
    public void loop()
    {
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








