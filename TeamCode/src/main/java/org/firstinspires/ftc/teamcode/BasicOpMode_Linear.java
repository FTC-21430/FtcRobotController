/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor liftMotor = null;


    Servo servoL;
    Servo servoR;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_Front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_Back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_Front");
        rightBackMotor = hardwareMap.get(DcMotor.class,"right_Back");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        double position_L = 1;
        double position_R = 0;
        servoL = hardwareMap.get(Servo.class, "left_hand");
        servoR = hardwareMap.get(Servo.class, "right_hand");
        servoL.setPosition(1);
        servoR.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double slide = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            boolean slowMode = gamepad1.right_bumper;
            boolean stick = gamepad2.b;
            boolean highJunction = gamepad2.dpad_up;
            boolean mediumJunction = gamepad2.dpad_right;
            boolean lowJunction = gamepad2.dpad_down;
            boolean groundJunction = gamepad2.dpad_left;
            float fastMode = gamepad1.right_trigger;




            //double turn  =  gamepad2.right_stick_x;


            leftFrontPower    =Range.clip(drive + slide + turn, -1.0, 1.0) ;
            leftBackPower  =Range.clip(drive - slide + turn,-1.0, 1.0 );
            rightFrontPower   =Range.clip(drive - slide - turn, -1.0, 1.0) ;
            rightBackPower  =Range.clip(drive + slide - turn, -1.0, 1.0);


            liftMotor.setTargetPosition(0);

            if(groundJunction){
                liftMotor.setTargetPosition(0);
            }
            if(lowJunction){
                liftMotor.setTargetPosition(2095);
            }
            if(mediumJunction){
                liftMotor.setTargetPosition(3619);
            }
            if(highJunction){
                liftMotor.setTargetPosition(5334);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            //leftPower  = -gamepad1.left_stick_y ;
            //rightPower = -gamepad1.right_stick_y ;
if(gamepad1.dpad_up){
    leftFrontPower =1;
    leftBackPower=1;
    rightFrontPower=1;
    rightBackPower=1;
}
            if(gamepad1.dpad_left){
                leftFrontPower =-1;
                leftBackPower=1;
                rightFrontPower=1;
                rightBackPower=-1;
            }
            if(gamepad1.dpad_right){
                leftFrontPower =1;
                leftBackPower=-1;
                rightFrontPower=-1;
                rightBackPower=1;
            }
            if(gamepad1.dpad_down){
                leftFrontPower =-1;
                leftBackPower=-1;
                rightFrontPower=-1;
                rightBackPower=-1;
            }
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
            if (stick == true) {
                position_L =1;
                position_R = 0; 
            }
            if (stick == false) {
                position_L = 0.85;
                position_R = 0.150;
            }

            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);
            //Set the servo to the new position and pause;
            servoL.setPosition(position_L);
            servoR.setPosition(position_R);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Encoders", leftBackMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
