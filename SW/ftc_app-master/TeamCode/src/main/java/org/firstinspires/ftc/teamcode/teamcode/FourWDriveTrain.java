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

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Hashtable;
import java.util.Set;

public class FourWDriveTrain
{

    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; //4.0 ;     // For figuring circumference
    static final double PI = 3.1415;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 1.0;
    static final double radiusOfRobotRotationPath = 14;
    private double distanceFor360Degrees = 2 * PI * radiusOfRobotRotationPath;

    static final double TIMEOUTINSECONDS = 30;

    double motorSpeed = 0.5;

    // map motor name to motor
    HashMap<String, DcMotor> motorHashMap = new HashMap<String, DcMotor>();


    public FourWDriveTrain() {}

    public void registerMotor (String motorName, DcMotor motor)
    {
        motorHashMap.put(motorName,motor);
    }

    public DcMotor getMotor(String motorName)
    {
        return motorHashMap.get(motorName);
    }

    public void init()
    {
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        Set <String> motorNames = motorHashMap.keySet();

        for (String motorName: motorNames)
        {
            DcMotor motor = motorHashMap.get(motorName);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveToDirection(String direction, double motorSpeed, double distance)
    {
        HashMap<String, Double> motorDistanceMap = new HashMap<String, Double>();

        if (direction == "North")
        {
            motorDistanceMap.put("frontLeft",distance);
            motorDistanceMap.put("frontRight", -distance);
            motorDistanceMap.put("backRight", distance);
            motorDistanceMap.put("backLeft", distance);
        }
        if (direction == "NorthWest")
        {
            motorDistanceMap.put("frontLeft",0.0);
            motorDistanceMap.put("frontRight", -distance);
            motorDistanceMap.put("backRight", 0.0);
            motorDistanceMap.put("backLeft", distance);
        }
        if (direction == "West") {
            motorDistanceMap.put("frontLeft",-distance);
            motorDistanceMap.put("frontRight", 0.0);
            motorDistanceMap.put("backRight", distance);
            motorDistanceMap.put("backLeft", distance);
        }
        if (direction == "SouthWest") {
            motorDistanceMap.put("frontLeft",-distance);
            motorDistanceMap.put("frontRight", 0.0);
            motorDistanceMap.put("backRight", distance);
            motorDistanceMap.put("backLeft", 0.0);
        }
        if (direction == "South") {
            motorDistanceMap.put("frontLeft",-distance);
            motorDistanceMap.put("frontRight", distance);
            motorDistanceMap.put("backRight", distance);
            motorDistanceMap.put("backLeft", -distance);
        }
        if (direction == "SouthEast") {
            motorDistanceMap.put("frontLeft",0.0);
            motorDistanceMap.put("frontRight", distance);
            motorDistanceMap.put("backRight", 0.0);
            motorDistanceMap.put("backLeft", -distance);
        }
        if (direction == "East") {
            motorDistanceMap.put("frontLeft",distance);
            motorDistanceMap.put("frontRight", distance);
            motorDistanceMap.put("backRight", -distance);
            motorDistanceMap.put("backLeft", -distance);
        }
        if (direction == "NorthEast") {
            motorDistanceMap.put("frontLeft",distance);
            motorDistanceMap.put("frontRight", 0.0);
            motorDistanceMap.put("backRight", -distance);
            motorDistanceMap.put("backLeft", 0.0);
        }
        if (direction == "noDirection") {
            motorDistanceMap.put("frontLeft",0.0);
            motorDistanceMap.put("frontRight", 0.0);
            motorDistanceMap.put("backRight", 0.0);
            motorDistanceMap.put("backLeft", 0.0);
        }
        if (direction == "Clockwise") {
            motorDistanceMap.put("frontLeft",distance);
            motorDistanceMap.put("frontRight", distance);
            motorDistanceMap.put("backRight", distance);
            motorDistanceMap.put("backLeft", distance);
        }
        if (direction == "CounterClockWise") {
            motorDistanceMap.put("frontLeft",-distance);
            motorDistanceMap.put("frontRight", -distance);
            motorDistanceMap.put("backRight", -distance);
            motorDistanceMap.put("backLeft", -distance);
        }
    }

    public void moveMotor(String motorName, double motorSpeed, double distance )
    {
        while (distance >= 0)
        {
            if (motorName.equalsIgnoreCase("frontLeft")) {
                frontLeft.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("frontRight")) {
                frontRight.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("backRight")) {
                backRight.setPower(motorSpeed);
            } else if (motorName.equalsIgnoreCase("backLeft")) {
                backLeft.setPower(motorSpeed);
            }
        }
    }

    public void moveMotor(HashMap<String, Double> motorDistanceMap, double speed)
    {
        encoderDriveInches(motorDistanceMap, speed, 30000);
    }

    public void encoderDriveInches(
            HashMap<String, Double> motorDistanceMap,
            double motorSpeed,
            double timeoutS)
    {

        Set <String> motorNames = motorDistanceMap.keySet();

        for (String motorName: motorNames)
        {
            DcMotor motor = motorHashMap.get(motorName);
            Double motordistance = motorDistanceMap.get(motorName);

            int newTargetDistance = motor.getCurrentPosition() + (int) (motordistance * COUNTS_PER_INCH);
            motor.setTargetPosition(newTargetDistance);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(Math.abs(motorSpeed));
        }
    }

    public void moveMotorDistance(DcMotor motor, double distanceInInches, double motorSpeed)
    {
        if (motor != null)
        {
            int targetPosition = motor.getCurrentPosition() + (int) (distanceInInches * COUNTS_PER_INCH);
            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(motorSpeed));
        }
    }
}
