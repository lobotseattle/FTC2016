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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import static java.lang.Thread.sleep;

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

@Autonomous(name="DCMotor omnibot gamepad opMode auto test", group="TestGroup")
public class DCMotor_omnibot_gamepad_opMode_auto_test  extends OpMode_Robot
{
    boolean state = false;

    double motorSpeed = 0.3;
    double turnSpeed = 0.1;

    @Override
    public void startActions()
    {
        super.startActions();
        telemetry.addData("Status", "Inside start actions: Derived class");
        telemetry.update();
        sleep(2000);

        if ( actionStack.empty()) return;
        IRobotAction action = (IRobotAction) actionStack.firstElement();

        Class c = action.getClass();
        telemetry.addData("Action", action.toString());
        telemetry.addData("Action Class", c.toString());
        telemetry.update();

        if ( action instanceof GoStraightRobotAction )
        {
            // cast the generic action into go straight
            GoStraightRobotAction robotAction = (GoStraightRobotAction) action;
            telemetry.addData("Status", "currentActionIndex: " + currentActionIndex);
            telemetry.update();
            driveRobotInDirection(robotAction.Direction, robotAction.Speed, robotAction.Distance);
        }

        else if (action instanceof TurnRobotAction )
        {
            TurnRobotAction robotAction = (TurnRobotAction) action;

            telemetry.addData("Status", "currentActionIndex: " + currentActionIndex);
            telemetry.update();
            rotateRobot(robotAction.MotorSpeed, robotAction.Degrees);
        }
        else if ( action instanceof ShootBallAction )
        {
            ShootBallAction shootAction = (ShootBallAction)action;
            // do the stuff here
        }
        else if (action instanceof SleepAction )
        {
            SleepAction sleepAction = (SleepAction) action;
            // do the stuff here
            sleep(sleepAction.milliseconds);
        }

        telemetry.addData("Action","Complete");
        actionStack.removeElementAt(0);


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
        telemetry.addData("Registering","Started");
        telemetry.update();
        actionStack.push(new GoStraightRobotAction("North", 1, 12));
        actionStack.push( new TurnRobotAction(1,180));
        actionStack.push ( new ShootBallAction() );
        actionStack.push( new GoStraightRobotAction ("East", 1, 12));
        actionStack.push( new SleepAction(500));
        actionStack.push (new GoStraightRobotAction ("East", 1, 12));
        actionStack.push ( new SleepAction(500));
        actionStack.push (  new GoStraightRobotAction("East",1,12) );
        actionStack.push ( new GoStraightRobotAction ("South",1,12) );
        actionStack.push( new GoStraightRobotAction ("West",1,12));
        telemetry.addData("Registering","Complete");
        telemetry.addData("Action count", actionStack.size());
        telemetry.update();

        //telemetry.addData("StartActions", "Derived class Called");
        //telemetry.update();
        double distance = 12;

//        //R1 and B1//
//    int i=0;
//    actions[i] = new GoStraightRobotAction("North", 1, 5);i++;
//    actions[i] = new TurnRobotAction(1, 180);i++;
//    actions[i] = new ShootBallAction();i++;
//    actions[i] = new GoStraightRobotAction ("East", 1, 7);i++;
//    actions[i] = new SleepAction(500);i++;
//    actions[i] = new GoStraightRobotAction ("East", 1, 0.083);i++;
//    //repeat until beacon turns color//
//    actions[i] = new GoStraightRobotAction("North", 1, 2);i++;
//    actions[i] = new SleepAction(500);i++;
//    actions[i] = new GoStraightRobotAction("East",1,0.083);i++;
//    actions[i] = new GoStraightRobotAction ("South",1,6);i++;
//    actions[i] = new GoStraightRobotAction ("West",1,1);i++;

/*

    //R2 and B2//
    actions[1] = new GoStraightRobotAction("North", 1, 5);
    actions[2] = new TurnRobotAction(1, 180);
    actions[3] = new
    //turn on shooter motor//
    actions[4] = new GoStraightRobotAction ("East", 1, 5);
    actions[5] = sleep(500);
    actions[6] = new GoStraightRobotAction ("East", 1, 0.083);
    //repeat until beacon turns color//
    actions[7] = new GoStraightRobotAction("North", 1, 2);
    actions[8] = sleep(500)
    actions[9] = new GoStraightRobotAction("East",1,0.083);
    actions[10] = new GoStraightRobotAction ("South",1,6);
    actions[11] = new GoStraightRobotAction ("West",1,1);
*/
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

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}








