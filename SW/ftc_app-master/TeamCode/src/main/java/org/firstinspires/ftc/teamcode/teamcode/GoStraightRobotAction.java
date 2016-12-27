package org.firstinspires.ftc.teamcode.teamcode;

import android.graphics.Path;

/**
 * Created by uma on 12/26/2016.
 */

public class GoStraightRobotAction implements IRobotAction {
    RobotActionModes robotActionMode = RobotActionModes.GoStraight;
    public String Direction;
    public double Speed ;
    public double Distance;

    public GoStraightRobotAction(String direction, double motorSpeed, double distance)
    {
        this.Direction = direction;
        this.Speed = motorSpeed;
        this.Distance = distance;
    }
}
