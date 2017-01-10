package org.legionofbot.ftc;

public class TurnRobotAction implements IRobotAction {
    RobotActionModes robotActionMode = RobotActionModes.Turn;
    public double MotorSpeed ;
    public int Degrees;

    public TurnRobotAction(double turnSpeed, int degrees)
    {
        this.robotActionMode = RobotActionModes.Turn;
        this.MotorSpeed = turnSpeed;
        this.Degrees = degrees;
    }
}
