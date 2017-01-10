
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


//@TeleOp(name="DCMotor omnibot gamepad dpad opmode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_omnibot_dpad_opmode extends OpMode {

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

    @Override
    public void init()
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
    public void loop()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

       // initialize();

        // Wait for the game to start (driver presses PLAY)
       // waitForStart();

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive())
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
                motorE.setPower(-gearMotorSpeed);
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
    /*public void moveMotor(String motorName, double motorSpeed, double distance )
    {
        while(distance >= 0 )
        {
            if(motorName.equalsIgnoreCase("frontLeft")) {
                motorA.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("frontRight")) {
                motorB.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("backRight")) {
                motorC.setPower(motorSpeed);
            } else if(motorName.equalsIgnoreCase("backLeft")) {
                motorD.setPower(motorSpeed);
            }
            else if(motorName.equalsIgnoreCase("motorE")) {
                motorE.setPower(motorSpeed);
            }

        }
    }*/

}