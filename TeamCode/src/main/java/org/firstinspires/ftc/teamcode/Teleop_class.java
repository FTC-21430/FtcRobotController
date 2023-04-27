package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class Teleop_class extends Robot {
    int LiftManual = 0;
    boolean calabrate_Lift = false;
    double Target = 0;
    double error = 0;
    double current = 0;
    boolean leftTurnold = false;
    boolean rightTurnold = false;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    double drive;
    double slide;
    double turn;
    double LiftAdd;
    boolean slowMode;
    boolean Intake;
    boolean highJunction;
    boolean mediumJunction;
    boolean lowJunction;
    boolean groundJunction;
    float fastMode;
    boolean upStack;
    boolean TurnLeft;
    boolean TurnRight;
    public void UpdateControls(){
         drive = -gamepad1.left_stick_y;
         slide = gamepad1.left_stick_x;
         turn = gamepad1.right_stick_x;
         LiftAdd = gamepad2.left_stick_y;
         slowMode = gamepad1.right_bumper;
         Intake = gamepad2.b;
         highJunction = gamepad2.dpad_up;
         mediumJunction = gamepad2.dpad_right;
         lowJunction = gamepad2.dpad_left;
         groundJunction = gamepad2.dpad_down;
         fastMode = gamepad1.right_trigger;
         upStack = gamepad2.left_bumper;
    }
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;

    public void IMUstuffs(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        current = orientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
    }
    public void IMUReset(){
        telemetry.addData("Yaw", "Reset" + "ing\n");
        imu.resetYaw();
        Target = 0;
    }
    public void ProportionalFeedbackControl(){
        error = Wrap((Target - current));
        if (gamepad1.right_stick_x != 0){
            imu.resetYaw();
            Target = 0;
        }

        turn -= error/20;
    }
    double Wrap(double angle){
        while(angle > 180){
            angle -= 360;
        }
        while(angle < -180){
            angle += 360;
        }
        return angle;
    }
    public void GridRunner(){
        if(gamepad1.dpad_up){
            drive = 1;
            slide = 0;
        }
        if(gamepad1.dpad_left){
            drive = 0;
            slide = -1;
        }
        if(gamepad1.dpad_right){
            drive = 0;
            slide = 1;
        }
        if(gamepad1.dpad_down){
            drive = -1;
            slide = 0;
        }
    }
    public void straferAlgorithm(){
        leftFrontPower = Range.clip(drive + slide + turn, -1.0, 1.0);
        leftBackPower  =Range.clip(drive - slide + turn,-1.0, 1.0 );
        rightFrontPower=Range.clip(drive - slide - turn, -1.0, 1.0);
        rightBackPower =Range.clip(drive + slide - turn, -1.0, 1.0);

    }
    public void LiftControl(){
        if (LiftAdd <= -0.2){
            LiftManual = LiftManual + 5;
            if (LiftManual >= 1851) LiftManual = 1850;
            if (LiftManual <= 1) LiftManual = 2;
            liftMotor.setTargetPosition(LiftManual);
            //go up
        }
        if (LiftAdd >= 0.2) {
            LiftManual = LiftManual - 5;
            if (LiftManual >= 1851) LiftManual = 1850;
            if (LiftManual <= 1) LiftManual = 2;
            liftMotor.setTargetPosition(LiftManual);


            // go down
        }
        if (upStack && !upStack_old ){
            // go up
            LiftManual += 60;
            if (LiftManual >= 1851) LiftManual = 1850;
            if (LiftManual <= 1) LiftManual = 2;
            liftMotor.setTargetPosition(LiftManual);

        }
        upStack_old = upStack;
        if (calabrate_Lift == true && liftMotor.getCurrentPosition() <= 100){

            if (digitalTouch.getState() == false){
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(-0.2);

            }
            if (digitalTouch.getState() == true || runtime.seconds() >= 3){
                liftMotor.setPower(0);
                liftMotor.setTargetPosition(0);
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(Math.abs(1));
                calabrate_Lift = false;
            }
        }else{
            liftMotor.setPower(Math.abs(1));
        }
        if (groundJunction) {
            GroundLift();
            LiftManual = 50;
            runtime.reset();
            calabrate_Lift = true;
        }
        if (lowJunction) {
            liftMotor.setTargetPosition(774);
            //liftMotor.setTargetPosition(2600);
            LiftManual = 774;
        }

        if (mediumJunction) {
            liftMotor.setTargetPosition(1302);
            // liftMotor.setTargetPosition(4000);
            rightPressed = true;
            LiftManual = 1302;
        }
        if (highJunction) {
            liftMotor.setTargetPosition(1820);
            LiftManual = 1820;
            //liftMotor.setTargetPosition(5700);
        }
    }
    public void speedControl(){
        leftFrontPower=leftFrontPower / 2;
        leftBackPower = leftBackPower / 2;
        rightFrontPower = rightFrontPower / 2;
        rightBackPower = rightBackPower / 2;
        if(fastMode==1){
            leftFrontPower=leftFrontPower * 2;
            leftBackPower = leftBackPower * 2;
            rightFrontPower = rightFrontPower * 2;
            rightBackPower = rightBackPower * 2;
        }
        if(slowMode){
            leftFrontPower=leftFrontPower / 2;
            leftBackPower = leftBackPower / 2;
            rightFrontPower = rightFrontPower / 2;
            rightBackPower = rightBackPower / 2;
        }
    }
    public void setMotorPower(){
        // Send calculated power to wheels
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
        //Set the servo to the new position and pause;
    }
}
