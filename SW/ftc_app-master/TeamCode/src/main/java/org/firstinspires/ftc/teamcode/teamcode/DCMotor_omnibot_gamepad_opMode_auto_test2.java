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

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

@Autonomous(name="DCMotor omnibot gamepad opMode auto test2", group="Opmode")
public class DCMotor_omnibot_gamepad_opMode_auto_test2 extends OpMode_Robot
{
    boolean state = false;

    double motorSpeed = 0.3;
    double turnSpeed = 0.1;


    @Override
    public void startActions(Telemetry telemetry)
    {
        telemetry.addData("Status", "currentActionIndex:" + currentActionIndex);
        telemetry.update();

       // rotateRobot(0.1, 90);

        if(currentActionIndex >= actions.length) {
            return;
        }

        if(actions[currentActionIndex].getClass().getSimpleName().contains("GoStraightRobotAction"))
        {
            GoStraightRobotAction action = (GoStraightRobotAction) actions[currentActionIndex];
            driveRobotInDirection(action.Direction, action.Speed, action.Distance);
        }
        else if(actions[currentActionIndex].getClass().getSimpleName().contains("TurnRobotAction"))
        {
            TurnRobotAction action = (TurnRobotAction) actions[currentActionIndex];
            rotateRobot(action.MotorSpeed, action.Degrees);
        }
        else if(actions[currentActionIndex].getClass().getSimpleName().contains("SleepAction"))
        {
            SleepAction action = (SleepAction) actions[currentActionIndex];
            sleepRobot(action.milliSeconds);
        }
        /*switch(actions[currentActionIndex].robotActionMode)
        {
            case RobotActionModes.GoStraight:
                moveToDirection(actions[currentActionIndex].Direction, actions[currentActionIndex].MotorSpeed, actions[currentActionIndex].Distance);
                break;
            default:
                break;
        }*/
    }

    @Override
    public void registerActions()
    {
        actions = new IRobotAction[8];

        //telemetry.addData("StartActions", "Derived class Called");
        //telemetry.update();
        double distance = 12;

        actions[0] = new TurnRobotAction(turnSpeed, 45);
        actions[1] = new SleepAction(1000);
        actions[2] = new TurnRobotAction(turnSpeed, -90);
        actions[3] = new SleepAction(1000);
        actions[4] = new GoStraightRobotAction("North", motorSpeed, distance);
        actions[5] = new SleepAction(1000);
        actions[6] = new GoStraightRobotAction("South", motorSpeed, distance);
        actions[7] = new SleepAction(1000);


       /* rotateRobot(90);
        rotateRobot(-90);
        rotateRobot(9);
        rotateRobot(-10);
        rotateRobot(15);
        rotateRobot(275);
        rotateRobot(365);*/

        //        moveToDirection("Clockwise", motorSpeed, distance);
  //      moveToDirection("CounterClockWise", motorSpeed,distance);
        //actions[0] = moveToDirection("North", motorSpeed, distance);
        //actions[1] = moveToDirection("South", motorSpeed, distance);

        /*actions[0] = new GoStraightRobotAction("North", motorSpeed, distance);
        actions[1] = new GoStraightRobotAction("South", motorSpeed, distance);
        actions[2] = new GoStraightRobotAction("East", motorSpeed, distance);
        actions[3] = new GoStraightRobotAction("West", motorSpeed, distance);
        actions[4] = new GoStraightRobotAction("NorthEast", motorSpeed,distance);
        actions[5] = new GoStraightRobotAction("SouthWest", motorSpeed, distance);
        actions[6] = new GoStraightRobotAction("SouthEast", motorSpeed,distance);*/
       /* actions[7-x] = new GoStraightRobotAction("NorthWest", motorSpeed,distance);
        actions[8-x] = new TurnRobotAction(turnSpeed, 90);
        actions[9-x] = new TurnRobotAction(turnSpeed, -90);
        actions[10-x] = new TurnRobotAction(turnSpeed, 9);
        actions[11-x] = new TurnRobotAction(turnSpeed, -10);
        actions[12-x] = new TurnRobotAction(turnSpeed, 15);
        actions[13-x] = new TurnRobotAction(turnSpeed, 275);
        actions[14-x] = new TurnRobotAction(turnSpeed, 365);*/

        //actions = new IRobotAction[1];
        //actions[0] = new TurnRobotAction(turnSpeed, 90);

      //  wait(3000);
    /*    moveToDirection("South", motorSpeed,distance);
        //wait(3000);


        moveToDirection("NorthEast", motorSpeed,distance);
        //sleep(3000);

        moveToDirection("SouthWest", motorSpeed, distance);
        //sleep(3000);

        moveToDirection("East", motorSpeed, distance);
        //sleep(3000);

        moveToDirection("West", motorSpeed, distance);
        //sleep(3000);

        moveToDirection("SouthEast", motorSpeed,distance);
       // sleep(3000);

        moveToDirection("NorthWest", motorSpeed,distance);
        //sleep(3000);*/
    }

}








