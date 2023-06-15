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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicOpMode_Linear extends Teleop_class {
    // Wait for the game to start (driver presses PLAY)
    @Override
    public void runOpMode() {
          Init();
      waitForStart();
      runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            UpdateControls();
        IMUstuffs();
            if (gamepad1.y) {
              IMUReset();
            }
          if (TurnLeft && !leftTurnold){
              Target -= 90;
          }
          if (TurnRight && !rightTurnold){
              Target += 90;
          }
            leftTurnold = gamepad1.b;
          rightTurnold = gamepad1.x;
          ProportionalFeedbackControl();
           GridRunner();
           straferAlgorithm();
            telemetry.addData("error", error);
            telemetry.addData("turn", turn);
            if (!calabrate_Lift) {
                liftMotor.setPower(Math.abs(1));
            }
            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
            LiftControl();
           speedControl();
            if (Intake == true) {
               IntakeOpen();
            }
            if (Intake == false) {
                IntakeClose();
            }
           setMotorPower();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Encoders", leftBackMotor.getCurrentPosition());
            telemetry.addData(" lift Encoders", liftMotor.getCurrentPosition());
            telemetry.addData("Right pressed", rightPressed);
            telemetry.addData("Lift manual add", LiftAdd);
            telemetry.addData("cala_lift", calabrate_Lift);
            telemetry.update();
        }
    }
}