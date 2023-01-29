package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;


//@Autonomous(name = "RedAuton1NoTensorFlow", group = "Linear Opmode")
// @TeleOp(...) is the other common choice
 //@Disabled
public class RedAuton1NoTensorFlow extends LinearOpMode {

    // Declare Devices
    DcMotor TopRight = null;
    DcMotor TopLeft = null;
    DcMotor BottomRight = null;
    DcMotor BottomLeft = null;
    DcMotor ArmMotor = null;
    DcMotor ArmMotor2 = null;


    private double linearSlideScale = 33.21;

    private DcMotor LinearSlideMotor = null;
    private int slidePos = 0;

    int armPos = 0;

    private Servo servoarm = null;
    private boolean holdRequest = false;



    //linear slide position
    private int linearSlideLow = 15;
    private int linearSlideMed = 24;
    private int linearSlideHigh = 35;
    // drive motor position variables
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    private int armCounter = 0;
    //private int testPos;

    // operational constants
    private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.3; // medium speed
    private double slow = 0.1; // slow speed
    private double clicksPerInch = 87.5; // empirically measured
    private double clicksPerDeg = 21.94; // empirically measured
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above
    private boolean isReleased = false;
    private boolean isLatched = false;
    private boolean isIn = false;


    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 386.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);


        // Initialize the hardware variables.
        TopLeft = hardwareMap.dcMotor.get("TopLeft");
        TopRight = hardwareMap.dcMotor.get("TopRight");
        BottomLeft = hardwareMap.dcMotor.get("BottomLeft");
        BottomRight = hardwareMap.dcMotor.get("BottomRight");

        // The right motors need reversing
        TopRight.setDirection(DcMotor.Direction.REVERSE);
        TopLeft.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.FORWARD);


        // Set the drive motor run modes:
        TopLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TopRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //OutakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servoarm = hardwareMap.get(Servo.class, "servoarm"); //write THIS name into the configuration

/*
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);

        ArmMotor2 = hardwareMap.get(DcMotor.class, "ArmMotor2");
        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor2.setDirection(DcMotor.Direction.FORWARD);
*/
        LinearSlideMotor = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        //close the servo
        //  servoOpen(servoarm);
        slideMotorUp(linearSlideMed, medium);
        // servoOpen(servoarm);
        // liftArmUp(ArmMotor, ArmMotor2);
        //wake up the robot
        //  moveForward(1,medium);
        //move right to get away from the cone infront
        //  moveRight(14, medium);
        //move forward to get closer to the closest medium pole
        //  moveForward(13,medium);
        //move right to get to the position near the pole
        //  moveRight(-5, medium);
        // servoOpen(servoarm,1);


        /*
        //move left to go to pick up the cone
        moveRight(-18, medium);

        moveForward(15,medium);
        //turn clockwise toward the cone
        turnClockwise(90,medium);
        //move toward the cone
        moveRight(13,medium);
        //go to pick up the cone
        moveForward(2, medium);
        //moveForward(3,medium);
*/

        //moving the servo
        // servoMove(servoarm);
        //deposit cone into the pole
        //-----------------------------------------------------
        //CODE HERE
        //-----------------------------------------------------
        //move backwards against the wall after depositing
        //  moveForward(-14, medium);
        //move left to the red carousel (extra code for positioning may be necessary)
        //  moveRight(-22, medium);
        //turn carousel so ducks will land on playing field
        //----------------------------------------------------
        //CODE HERE
        //----------------------------------------------------
        //move right to park completely in warehouse
        //   moveRight(49, medium);



        //this is a 90 degree turn
        //turnClockwise(-23, medium);


    }

    private void moveForward(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();
        //testPos = TestMotor.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lrPos += howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;
        //testPos += howMuch * clicksPerInch;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        //TestMotor.setTargetPosition(testPos);

        TopLeft.setPower(speed); //WORKS
        TopRight.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d : %7d : %7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d : %7d : %7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
    }

    private void moveRight(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        TopRight.setPower(speed);
        TopLeft.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);

    }

    private void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();

        // calculate new target
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        TopLeft.setPower(speed);
        TopRight.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        TopLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TopRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
    }

    private void moveToLine(int howMuch, double speed) {
        // howMuch is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // fetch motor positions
        lfPos = TopLeft.getCurrentPosition();
        rfPos = TopRight.getCurrentPosition();
        lrPos = BottomLeft.getCurrentPosition();
        rrPos = BottomRight.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        TopLeft.setTargetPosition(lfPos);
        TopRight.setTargetPosition(rfPos);
        BottomLeft.setTargetPosition(lrPos);
        BottomRight.setTargetPosition(rrPos);
        TopLeft.setPower(speed);
        TopRight.setPower(speed);
        BottomLeft.setPower(speed);
        BottomRight.setPower(speed);

        // wait for move to complete
        while (TopLeft.isBusy() && TopRight.isBusy() &&
                BottomLeft.isBusy() && BottomRight.isBusy()) {
            //if (mrOds.getLightDetected() > lineThreshold) break;

            // Display it for the driver.
            telemetry.addLine("Move To Line");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", TopLeft.getCurrentPosition(),
                    TopRight.getCurrentPosition(), BottomLeft.getCurrentPosition(),
                    BottomRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);

    }
    /*
    //In Progress: Need to make it suitable for autonomous
    private void liftArmUp(DcMotor ArmMotor, DcMotor ArmMotor2){
        holdRequest = false;

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmMotor.setPower(0.3);
        ArmMotor2.setPower(-0.3);
        while (armCounter != 500){
            armCounter = armCounter+1;
            telemetry.addData("ARM COUNTER: ", armCounter);
            telemetry.addData("Current Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }

    }
    private void liftArmDown(DcMotor ArmMotor, DcMotor ArmMotor2,Servo servoarm){
        holdRequest = false;

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmMotor.setPower(0.3);
        ArmMotor2.setPower(-0.3);

        while (armCounter != 50){
            armCounter = armCounter-1;
            telemetry.addData("ARM COUNTER: ", armCounter);
            telemetry.addData("Current Position arm1:  arm2:", "%7d :%7d ", ArmMotor.getCurrentPosition(),ArmMotor2.getCurrentPosition());
            telemetry.update();
        }
        servoarm.setDirection(Servo.Direction.FORWARD);
        servoarm.setPosition(0);

    }
    */

    private void slideMotorUp(int howMuch,double speed) {
        slidePos = LinearSlideMotor.getCurrentPosition() ;
        slidePos+=howMuch * clicksPerInch ;
        // slidePos += howMuch * clicksPerInch;
        LinearSlideMotor.setTargetPosition(slidePos);
        LinearSlideMotor.setPower(speed);
        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Current Position arm: ", ":%7d", LinearSlideMotor.getCurrentPosition());
        telemetry.update();
        //   while(armCounter != 799){
        //     armCounter+=1;
        //    telemetry.addData("ARM COUNTER: ", armCounter);
        //   telemetry.addData("Current Position arm: ", ":%7d", LinearSlideMotor.getCurrentPosition());
        //  telemetry.update();
        //}
        LinearSlideMotor.setPower(0);

        //
    }


    // end of slideMotor
    //In Progress: Need to make it suitable for autonomous
    private void servoOpen(Servo servoarm) {
        //   if (val == 0)
        //   {
        if (servoarm != null) {
            servoarm.setDirection(Servo.Direction.FORWARD);
            servoarm.setPosition(-0.5);
        }


    }
    //  }
        /*
        else if(val == 1) {
            if (servoarm != null) {
                servoarm.setDirection(Servo.Direction.FORWARD);
                servoarm.setPosition(0);
            }
        }
        else{
            if (servoarm != null) {

                servoarm.setPosition(0);
            }
        }
*/

    //
} // end of carouselMotor


