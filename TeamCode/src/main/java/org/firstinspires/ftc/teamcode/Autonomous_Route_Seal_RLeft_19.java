

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous_Route_Seal_RLeft_19", group="Monkey")
public class Autonomous_Route_Seal_RLeft_19 extends LinearOpMode {
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
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_Front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_Back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_Front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_Back");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        double postion_L = 1;
        double postion_R = 1;

        servoL = hardwareMap.get(Servo.class, "left_hand");
        servoR = hardwareMap.get(Servo.class, "right_hand");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoL.setPosition((0.72));
        servoR.setPosition((0.33));


        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(1);


        waitForStart();
        runtime.reset();
        //L=0.85 R=0.15 is open and L=0 R=1 is closed

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        float ZONE = 1;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        servoL.setPosition(0.65);
        servoR.setPosition(0.4);

        //Power Source






        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();




        encoderDrive(0.5, 1.8, 1.8, -1.8, -1.8,5);
            //Sideways to the Right
//        rightBackMotor.setPower(0.85);
//        leftFrontMotor.setPower(0.85);
//        leftBackMotor.setPower(-0.85);
//        rightFrontMotor.setPower(-0.85);
//
//        sleep(10);
//
        encoderDrive(0.5,2.0,2.0,2.0,2.0,5);
//        //Forwards
//        leftBackMotor.setPower(0.35);
//        rightBackMotor.setPower(0.35);
//        rightFrontMotor.setPower(0.35);
//        leftFrontMotor.setPower(0.35);
//
//        sleep(845);

        encoderDrive(0.5, 1.4, 1.4, -1.4, -1.4, 5);
//        //slide to the Right
//
//        leftBackMotor.setPower(-0.85);
//        rightBackMotor.setPower(0.85);
//        rightFrontMotor.setPower(-0.85);
//        leftFrontMotor.setPower(0.85);
        liftMotor.setTargetPosition(1741);
        sleep(5000);
        encoderDrive(0.3, 0.6,0.6,0.6,0.6,5 );
        //forwards

        servoL.setPosition(0.81);
        servoR.setPosition(0.312);
        encoderDrive(0.75,2.0,2.0,2.0,2.0,5);

//        sleep(625);

        sleep(700);
        servoL.setPosition(0.81);
        servoR.setPosition(0.312);
        //open
        encoderDrive(0.5,-0.54, -0.54, -0.54, -0.54, 5);
//        Backwards

        sleep(700);
        servoL.setPosition((0.65));
        servoR.setPosition(0.40);
        //closed
        liftMotor.setTargetPosition(0);
        //   encoderDrive(0.75, -1.5, -1.5, 1.5, 1.5, 5);
        // encoderDrive(0.3, 1.5,1.5,1.5,1.5,5 );
        stop();
if (ZONE==1){
    //move to zone 1
}
if(ZONE==2){
    //move to zone2
}
if (ZONE==3 ){
    //move to zone 3
}
      sleep( 3000);
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
