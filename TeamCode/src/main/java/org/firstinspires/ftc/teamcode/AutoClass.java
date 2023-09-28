package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class AutoClass extends Teleop_class {

    //VARAIBLE ZONE!!!


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
    public double scalingDouble;
    public double FrontLeftOld,FrontRightOld,BackLeftOld,BackRightOld;
    public float TESTfLeft, TESTfRight, TESTbLeft, TESTbRight;



public double startOfsetRadians = 0;
    public void RobotAngles(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
    RobotAngle = orientation.getYaw(AngleUnit.RADIANS);
        RobotAngle += startOfsetRadians;
    }
    public void UpdateOdometry(){

        DForward = (FrontRight + FrontLeft + BackRight + BackLeft)/4;
        DSideways = (-FrontRight + FrontLeft + BackRight - BackLeft)/4/1.2;
        RobotX = (InitX - DForward * Math.sin(RobotAngle)+ DSideways * Math.cos(RobotAngle));
        RobotY = (InitY + DForward * Math.cos(RobotAngle)+ DSideways * Math.sin(RobotAngle));
        InitX = RobotX;
        InitY = RobotY;
    }
    public void keepAtPoint(double Tx, double Ty) {

        distanceX = RobotX - Tx;
        distanceY = RobotY - Ty;

        PowerX = -distanceX * .55;
        PowerY = -distanceY * .4;

        PowerS = PowerX * Math.cos(-RobotAngle) - PowerY * Math.sin(-RobotAngle);
        PowerF = PowerX * Math.sin(-RobotAngle) + PowerY * Math.cos(-RobotAngle);


        scalingDouble = Math.max(Math.abs(PowerS),Math.abs(PowerF));
        if (scalingDouble >= 1){
            PowerF/= scalingDouble;
            PowerS/= scalingDouble;
        }
        drive = PowerF;
        slide = PowerS;
        straferAlgorithm();


    }
    public void RunToPoint(double TargetX, double TargetY){

        while(distanceCircle(TargetX,TargetY) > 0.2 & opModeIsActive())
        {
            telemetry.addData("distance",distanceCircle(TargetX,TargetY));
            telemetry.addData("Y", RobotY);
            telemetry.addData("X", RobotX);
            telemetry.addData("Angle", RobotAngle);
            telemetry.addData( "a motor", FrontLeft);
            telemetry.update();
            TESTfLeft = leftFrontMotor.getCurrentPosition();
            TESTfRight = rightFrontMotor.getCurrentPosition();
            TESTbLeft = leftBackMotor.getCurrentPosition();
            TESTbRight = rightBackMotor.getCurrentPosition();
            //UpdateControls();
//            drive = -gamepad1.left_stick_y;
//            slide = gamepad1.left_stick_x;
           turn = 0;
            LiftControl();
            IMUstuffs();

            RobotAngles();
            ProportionalFeedbackControl();
            UpdateEncoders();

            UpdateOdometry();
            keepAtPoint(TargetX, TargetY);
            GridRunner();

            straferAlgorithm();
            leftFrontPower=leftFrontPower / 2;
            leftBackPower = leftBackPower / 2;
            rightFrontPower = rightFrontPower / 2;
            rightBackPower = rightBackPower / 2;
            setMotorPower();


         }
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
         rightFrontMotor.setPower(0);
          rightBackMotor.setPower(0);

    }
    public double distanceCircle(double x, double y){
        return(Math.sqrt((x-RobotX)*(x-RobotX) + (y-RobotY)*(y-RobotY)));
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
