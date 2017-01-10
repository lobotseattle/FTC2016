
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


//@Autonomous(name="DCMotor omnibot gamepad auto test", group="Linear Opmode")
public class DCMotor_omnibot_gamepad_auto_test extends DCMotor_2Wheel_Encoder2_4Wheels
{
    boolean state = false;

    double motorSpeed = 0.3;
    @Override
    public void startActions()
    {

        //telemetry.addData("StartActions", "Derived class Called");
        //telemetry.update();
        double distance = 20;


        rotateRobot(90);
        rotateRobot(-90);
        rotateRobot(9);
        rotateRobot(-10);
        rotateRobot(15);
        rotateRobot(275);
        rotateRobot(365);


//        moveToDirection("Clockwise", motorSpeed, distance);
  //      moveToDirection("CounterClockWise", motorSpeed,distance);
/*
        moveToDirection("North", motorSpeed, distance);
        sleep(3000);
        moveToDirection("South", motorSpeed,distance);
        sleep(3000);


        moveToDirection("NorthEast", motorSpeed,distance);
        sleep(3000);

        moveToDirection("SouthWest", motorSpeed, distance);
        sleep(3000);

        moveToDirection("East", motorSpeed, distance);
        sleep(3000);

        moveToDirection("West", motorSpeed, distance);
        sleep(3000);

        moveToDirection("SouthEast", motorSpeed,distance);
        sleep(3000);

        moveToDirection("NorthWest", motorSpeed,distance);
        sleep(3000);
*/
    }

}








