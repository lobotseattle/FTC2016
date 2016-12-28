package org.firstinspires.ftc.teamcode.teamcode;

/**
 * Created by srikantr on 12/27/2016.
 */

public class SleepAction implements IRobotAction
{
    RobotActionModes robotActionMode = RobotActionModes.Sleep;

    public long milliseconds =0;
    public SleepAction(int milliseconds)
    {
        this.milliseconds = milliseconds;
    }
    public String toString()
    {
        return "Sleep Action";
    }
}
