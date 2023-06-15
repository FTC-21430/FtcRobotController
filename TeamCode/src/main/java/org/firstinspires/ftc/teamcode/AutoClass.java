package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class AutoClass extends Teleop_class {

    //VARAIBLE ZONE!!!
    public double RobotAngle = 0;
    public double RobotX, RobotY;
    public double InitX, InitY;
    public double DForward, DSideways;
    public double correctionFactor = 1.016723060905778;
    public double ticksPerRevolution = 537.7;
    public double MMPerRevolution = 96*Math.PI;
    public double MMPerInch = 25.4;
    public double FrontLeft = 0;
    public double FrontRight=0;
    public double BackLeft=0;
    public double BackRight=0;
    public double FrontLeftOld,FrontRightOld,BackLeftOld,BackRightOld;
    public float TESTfLeft, TESTfRight, TESTbLeft, TESTbRight;


public float startOfsetDegrees = 90;
    public void RobotAngles(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        RobotAngle = orientation.getYaw(AngleUnit.DEGREES);
        RobotAngle += startOfsetDegrees;
    }
    public void UpdateOdometry(){

        DForward = (FrontRight + FrontLeft + BackRight + BackLeft)/4;
        DSideways = (-FrontRight + FrontLeft + BackRight - BackLeft)/4;
        RobotX = (InitX + DForward * Math.cos(RobotAngle)+ DSideways * Math.cos(RobotAngle + 90));
        RobotY = (InitY + DForward * Math.sin(RobotAngle)+ DSideways * Math.sin(RobotAngle + 90));
        InitX = RobotX;
        InitY = RobotY;
    }
    public void UpdateEncoders(){
        FrontLeft = TESTfLeft;
        FrontRight = TESTfRight;
        BackLeft = TESTbLeft;
        BackRight = TESTbRight;

        FrontLeft -= FrontLeftOld;
        BackLeft -= BackLeftOld;
        FrontRight -= FrontRightOld;
        BackRight -= BackRightOld;

        FrontLeftOld = TESTfLeft;
        FrontRightOld = TESTfRight;
        BackLeftOld =  TESTbLeft;
        BackRightOld = TESTbRight;

        FrontLeft = FrontLeft/ticksPerRevolution;
        FrontLeft = MMPerRevolution*FrontLeft;
        FrontLeft = FrontLeft/MMPerInch;
        FrontLeft *= correctionFactor;

        FrontRight = FrontRight/ticksPerRevolution;
        FrontRight = MMPerRevolution*FrontRight;
        FrontRight = FrontRight/MMPerInch;
        FrontRight *= correctionFactor;

        BackRight = BackRight/ticksPerRevolution;
        BackRight = MMPerRevolution*BackRight;
        BackRight = BackRight/MMPerInch;
        BackRight *= correctionFactor;

        BackLeft = BackLeft/ticksPerRevolution;
        BackLeft = MMPerRevolution*BackLeft;
        BackLeft = BackLeft/MMPerInch;
        BackLeft *= correctionFactor;


        telemetry.addData("left front inches: ", FrontLeft);
        telemetry.addData("right front inches: ", FrontRight);
        telemetry.addData("left back inches: ", BackLeft);
        telemetry.addData("right back inches: ", BackRight);

    }
}
