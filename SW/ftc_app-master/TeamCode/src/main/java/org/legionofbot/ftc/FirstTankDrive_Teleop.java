package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name="K9bot: FirstTankDrive_Teleop", group="K9bot")
public class FirstTankDrive_Teleop extends OpMode {
    DcMotor leftWheel = null;
    DcMotor rightWheel = null;

    double leftWheelPower;
    double rightWheelPower;

    @Override
    public void init() {
        telemetry.addData("hi-this is test", 0.12);
        telemetry.update();
        leftWheel = hardwareMap.dcMotor.get("left_motor");
    //    rightWheel = hardwareMap.dcMotor.get("right_wheel");

        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        telemetry.addData("inside loop", 0.34);
        telemetry.update();
        telemetry.addData("inside loopleft", gamepad1.left_stick_y);
        telemetry.update();
        leftWheelPower = gamepad1.left_stick_y;
        //rightWheelPower = gamepad1.right_stick_y;
        leftWheel.setPower(0.5);
        leftWheel.setPower(leftWheelPower);
        telemetry.update();
      //  rightWheel.setPower(rightWheelPower);
    }
}
