
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DCMotor omnibot gamepad detectwheel", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class DCMotor_omnibot_detectwheel extends LinearOpMode
{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorA = null;
    DcMotor motorB = null;
    DcMotor motorC = null;
    DcMotor motorD = null;
    DcMotor motorE = null;
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
        motorE = hardwareMap.dcMotor.get("motor_shooter");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorA.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        motorB.setDirection(DcMotor.Direction.REVERSE);
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.REVERSE);
        motorE.setDirection(DcMotor.Direction.REVERSE);

        // boolean oldRBPressed = false;
        // boolean newRBPressed = false;
        // boolean isSpindleMotorOn = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            telemetry.addData("status", "Spinning motor A");
            telemetry.update();
            motorA.setPower(motorSpeed);
            sleep(4000);

            telemetry.addData("status", "Spinning motor B");
            telemetry.update();
            motorA.setPower(0);
            motorB.setPower(motorSpeed);
            sleep(4000);

            telemetry.addData("status", "Spinning motor C");
            telemetry.update();
            motorB.setPower(0);
            motorC.setPower(motorSpeed);
            sleep(4000);

            telemetry.addData("status", "Spinning motor D");
            telemetry.update();
            motorC.setPower(0);
            motorD.setPower(motorSpeed);
            sleep(4000);

            telemetry.addData("status", "Spinning motor E");
            telemetry.update();
            motorD.setPower(0);
            motorE.setPower(motorSpeed);
            sleep(4000);

            break;

            /*if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0)
            {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
            else {
                double angle = getGamepadAngle(gamepad1.right_stick_x, gamepad1.right_stick_y);
                String direction = getDirectionFromAngle(angle);
                moveToDirection(direction);
            }*/
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
                return 90;
            }
            else
            {
                return 270;
            }
        }

        tanValue = y / x ;
        double radians = Math.atan(tanValue);
        angle = (radians *180)/Math.PI;
// todo find the correct fn
        telemetry.addData("The angle is set to ",angle);
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
            motorD.setPower(-motorSpeed);
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








