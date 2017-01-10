
package org.legionofbot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


//@Autonomous(name="Omnibot OpMode Auto", group="Opmode")
public class Omnibot_OpMode_Auto extends OpMode_Robot
{
    boolean state = false;

    double motorSpeed = 0.3;
    double turnSpeed = 0.1;


    @Override
    public void startActions(Telemetry telemetry)
    {
        telemetry.addData("Status", "currentActionIndex:" + currentActionIndex);
        telemetry.update();

        if(currentActionIndex >= lstActions.size()) {
            return;
        }

        if(lstActions.get(currentActionIndex).getClass().getSimpleName().contains("GoStraightRobotAction"))
        {
            GoStraightRobotAction action = (GoStraightRobotAction) lstActions.get(currentActionIndex);
            driveRobotInDirection(action.Direction, action.Speed, action.Distance);
        }
        else if(lstActions.get(currentActionIndex).getClass().getSimpleName().contains("TurnRobotAction"))
        {
            TurnRobotAction action = (TurnRobotAction) lstActions.get(currentActionIndex);
            telemetry.addData("Status", "TurnRobot " + action.Degrees);
            telemetry.update();
            rotateRobot(action.MotorSpeed, action.Degrees);
        }
        else if(lstActions.get(currentActionIndex).getClass().getSimpleName().contains("SleepAction"))
        {
            SleepAction action = (SleepAction) lstActions.get(currentActionIndex);
            sleepRobot(action.milliSeconds, false);
        }
        else if(lstActions.get(currentActionIndex).getClass().getSimpleName().contains("ShootAction"))
        {
            ShootAction action = (ShootAction) lstActions.get(currentActionIndex);
            shooterMotor.setPower(action.Speed);
            sleepRobot(action.milliSeconds, true);
        }


    }

    @Override
    public void registerActions()
    {
        double distance = 12;

        lstActions = new ArrayList<IRobotAction>();


        lstActions.add(new TurnRobotAction(turnSpeed, 45));

        lstActions.add(new GoStraightRobotAction("North", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("South", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("East", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("West", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("NorthEast", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("SouthWest", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("SouthEast", motorSpeed, 12));
        lstActions.add(new SleepAction(2));
        lstActions.add(new GoStraightRobotAction("NorthWest", motorSpeed, 12));

    }

}








