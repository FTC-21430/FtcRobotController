

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous_Route_Monkey_Blue_16", group="Monkey")
public class Autonomous_Route_Monkey_Blue_Left_16 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_Front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_Back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_Front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_Back");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double slide = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.left_bumper;
        //double turn  =  gamepad1.right_stick_x;

        //Power Source
        leftFrontPower = Range.clip(drive + slide + turn, -1.0, 1.0);
        leftBackPower = Range.clip(drive - slide + turn, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - slide - turn, -1.0, 1.0);
        rightBackPower = Range.clip(drive + slide - turn, -1.0, 1.0);

        if (slowMode) {
            leftFrontPower = leftFrontPower / 2;
            leftBackPower = leftBackPower / 2;
            rightFrontPower = rightFrontPower / 2;
            rightBackPower = rightBackPower / 2;
        }

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        //leftPower  = -gamepad1.left_stick_y ;
        //rightPower = -gamepad1.right_stick_y ;
        if (slowMode) {
            leftFrontPower = leftFrontPower / 2;
            leftBackPower = leftBackPower / 2;
            rightFrontPower = rightFrontPower / 2;
            rightBackPower = rightBackPower / 2;


            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.update();


        encoderDrive(0.85, -1.6, -1.6, 1.6, 1.6,5);

            //Sideways to the Left
//        rightBackMotor.setPower(0.85);
//        leftFrontMotor.setPower(0.85);
//        leftBackMotor.setPower(-0.85);
//        rightFrontMotor.setPower(-0.85);
//
//        sleep(395);
//
        encoderDrive(0.35,0.9,0.9,0.9,0.9,5);
//        //Forwards
//        leftBackMotor.setPower(0.35);
//        rightBackMotor.setPower(0.35);
//        rightFrontMotor.setPower(0.35);
//        leftFrontMotor.setPower(0.35);
//
//        sleep(416);
//
        encoderDrive(-0.35,-1, -1, -1, -1, 5);
//        //Backwards
//        leftBackMotor.setPower(-0.35);
//        rightBackMotor.setPower(-0.35);
//        rightFrontMotor.setPower(-0.35);
//        leftFrontMotor.setPower(-0.35);
//
//        sleep(416);
//
        encoderDrive(0.85, 3.5, 3.5, -3.5, -3.5, 5);
//        //sideways to the Right
//
//        leftBackMotor.setPower(-0.85);
//        rightBackMotor.setPower(0.85);
//        rightFrontMotor.setPower(-0.85);
//        leftFrontMotor.setPower(0.85);
//
//        sleep(726);

        stop();
    }
    static final double     COUNTS_PER_FOOT  =  546.4480;
    public void encoderDrive(double speed,
                             double leftFrontMotorFoot, double rightBackMotorFoot, double leftBackMotorFoot, double rightFrontMotorFoot,
                             double timeoutS) {
        int newleftBackMotorTarget;
        int newletfFrontMotorTarget;
        int newrightFrontMotorTarget;
        int newrightBackMotorTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newleftBackMotorTarget = leftBackMotor.getCurrentPosition() + (int)(leftBackMotorFoot * COUNTS_PER_FOOT);
            newletfFrontMotorTarget= leftFrontMotor.getCurrentPosition() + (int)(leftFrontMotorFoot * COUNTS_PER_FOOT);
            newrightFrontMotorTarget = rightFrontMotor.getCurrentPosition() + (int)(rightFrontMotorFoot* COUNTS_PER_FOOT);
            newrightBackMotorTarget = rightBackMotor.getCurrentPosition() + (int)(rightBackMotorFoot * COUNTS_PER_FOOT);


            leftBackMotor.setTargetPosition(newleftBackMotorTarget);
            leftFrontMotor.setTargetPosition(newletfFrontMotorTarget);
            rightBackMotor.setTargetPosition(newrightBackMotorTarget);
            rightFrontMotor.setTargetPosition(newrightFrontMotorTarget);

            // Turn On RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftBackMotor.setPower(Math.abs(speed));
            leftFrontMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newletfFrontMotorTarget,  newleftBackMotorTarget, newrightBackMotorTarget, newrightFrontMotorTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftBackMotor.getCurrentPosition(), leftFrontMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
