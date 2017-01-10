
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


@Autonomous(name="DCMotor omnibot gamepad opMode auto test", group="Linear Opmode")
public class DCMotor_omnibot_gamepad_opMode_auto_test  extends OpMode_Robot
{
    boolean state = false;

    double motorSpeed = 0.3;
    double turnSpeed = 0.1;

    @Override
    public void startActions(Telemetry telemetry)
    {
        telemetry.addData("Status", "Inside start actions");
        telemetry.update();

        rotateRobot(0.1, 90);

        if(currentActionIndex >= lstActions.size()) {
            return;
        }

       // if(actions[currentActionIndex].robotActionMode == RobotActionModes.GoStraight)
        {
            GoStraightRobotAction action = (GoStraightRobotAction) lstActions.get(currentActionIndex);

            telemetry.addData("Status", "currentActionIndex: " + currentActionIndex);
            telemetry.update();
            driveRobotInDirection(action.Direction, action.Speed, action.Distance);
        }
        //else if(actions[currentActionIndex].robotActionMode == RobotActionModes.Turn)
        {
            TurnRobotAction action = (TurnRobotAction) lstActions.get(currentActionIndex);

            telemetry.addData("Status", "currentActionIndex: " + currentActionIndex);
            telemetry.update();
            rotateRobot(action.MotorSpeed, action.Degrees);
        }

    }

    @Override
    public void registerActions()
    {
        int x = 7;

        lstActions = new ArrayList<IRobotAction>();

        //telemetry.addData("StartActions", "Derived class Called");
        //telemetry.update();
        double distance = 12;
        
        //R1 and B1//
    lstActions.add(new GoStraightRobotAction("North", 1, 5));
        lstActions.add(new TurnRobotAction(1, 180));
        lstActions.add(new GoStraightRobotAction("West", 1, 0.083));
    //turn on shooter motor//
        lstActions.add(new GoStraightRobotAction ("East", 1, 7));
   // actions[5] = sleep(500);
        lstActions.add(new GoStraightRobotAction ("East", 1, 0.083));
    //repeat until beacon turns color//
        lstActions.add(new GoStraightRobotAction("North", 1, 2));
   // actions[8] = sleep(500);
        lstActions.add(new GoStraightRobotAction("East",1,0.083));
        lstActions.add(new GoStraightRobotAction ("South",1,6));
        lstActions.add(new GoStraightRobotAction ("West",1,1));



    //R2 and B2//
        lstActions.add(new GoStraightRobotAction("North", 1, 5));
        lstActions.add(new TurnRobotAction(1, 180));
    //actions[3] = new
    //turn on shooter motor//
        lstActions.add(new GoStraightRobotAction ("East", 1, 5));
    //actions[5] = sleep(500);
        lstActions.add(new GoStraightRobotAction ("East", 1, 0.083));
    //repeat until beacon turns color//
        lstActions.add(new GoStraightRobotAction("North", 1, 2));
    //actions[8] = sleep(500);
        lstActions.add(new GoStraightRobotAction("East",1,0.083));
        lstActions.add(new GoStraightRobotAction ("South",1,6));
        lstActions.add(new GoStraightRobotAction ("West",1,1));

    }

}








