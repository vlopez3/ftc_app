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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Talon Linear Blue", group="Pushbot")
 //@Disabled
public class TalonLinearBlue extends LinearOpMode {

    /* Declare OpMode members. */
    // HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     runtimeShoot = new ElapsedTime();
    private ElapsedTime     runtimeScoop = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // for ANDYMARK
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     WHITE_THRESHOLD = 0.2;
    static final double gray = 0.1;
    private ColorSensor colorSensor;
    OpticalDistanceSensor   lightSensor;



    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor scoop;
    DcMotor shoot;


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        scoop = hardwareMap.dcMotor.get("scoop");
        shoot = hardwareMap.dcMotor.get("shoot");
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");


        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(false);
        lightSensor.enableLed(true);

        // Set the LED in the beginning
       //colorSensor.enableLed(bLedOn);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d Scoop: %7d",
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition(), scoop.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        /*The sequence for the whole autonomous mode is:
        1. Move fwd
        2. Shoot
        3. Reload
        4. Shoot
        5. Fwd
        6. Turn
        7. Fwd to beacon
        8. Test Beacon
        9. Turn
        10. Fwd
        11. Turn
        12. Fwd to beacon
        13. Test Beacon
        14. Back
        15. Turn.
        16. Back to park


         */



        //Beginning the motion
        //1. Turns
        encoderDrive(0.5,33,33,5);
        //2. Shoot 1st ball
        shoot.setTargetPosition(-1099);
        shoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoot.setPower(0.4);
        sleep(200);
        //this while make the previous movement happens
        while (opModeIsActive() &&
                shoot.isBusy()) {
            telemetry.addData("Intermedia", "%7d :%7d Scoop: %7d",
                    motorLeft.getCurrentPosition(),
                    motorRight.getCurrentPosition(), shoot.getCurrentPosition());
            telemetry.update();
        }
        //3. Load
        scoop.setPower(-0.7);
        shoot.setPower(0);
        sleep(1500);

        //4. Shoot 2nd ball
        scoop.setPower(0);
        shoot.setTargetPosition(-1346);
        shoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoot.setPower(1);
        while (opModeIsActive() &&
                shoot.isBusy()) {
            telemetry.addData("Intermedia", "%7d :%7d Scoop: %7d",
                    motorLeft.getCurrentPosition(),
                    motorRight.getCurrentPosition(), shoot.getCurrentPosition());
            telemetry.update();
        }
//Heading for the Beacons
        //5. Move fwd
        encoderDrive(0.5,29,29,5);
        //6 Turn
        driveToPosition(0.4,-626,571,5);
        //7 Move fwd to touch the Beacon
        encoderDrive(0.4,15.5,15.5,5);

        //8. check beacon
        telemetry.addData("Color Detected", "Red %d Blue %d Green %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
        telemetry.update();
        sleep(2000);

        //8.1. If color right (Blue) then move back
       if(colorSensor.blue()>=4 ){

           telemetry.addData("Selected Blue", "Red %d Blue %d Green %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
           telemetry.update();
           encoderDrive(0.5,-23,-23,5);
           sleep(2000);
       }
       //8.2. If color wrong then move back and forward then back
        else {
           telemetry.addData("Selected Red", "Red %d Blue %d Green %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
           telemetry.update();
           encoderDrive(0.5,-10,-10,5);
           sleep(5000);
           encoderDrive(0.5,10,10,5);
           encoderDrive(0.5,-23,-23,5);

           }
        //9. Turn to move towards 2nd beacon
        driveToPosition(0.4,1046,-1078,5);
        //10. Move fwd to second beacon
        encoderDrive(0.5,50,50,5);
        //11. Turn to head beacon
        driveToPosition(0.4,-1096,1073,5);
        //12. move forward to press beacon
        encoderDrive(0.5,25,25,5);
        //13. check  2nd beacon
        telemetry.addData("ColorNumber", "Red %d Blue %d Green %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
        telemetry.update();

        //13.1. If color right (Blue) then move back
        if(colorSensor.blue()>=8 ){

            telemetry.addData("Blue", "Red %d Blue %d Green %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
            telemetry.update();
            encoderDrive(0.5,-32,-32,5);
            sleep(2000);
        }
        //13.2. If color wrong then move back and forward then back
        else {
            telemetry.addData("Red", "Red %d Blue %d Green %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
            telemetry.update();
            encoderDrive(0.5,-10,-10,5);
            sleep(5000);
            encoderDrive(0.5,10,10,5);
            encoderDrive(0.5,-32,-32,5);

        }
        //14. Turn to park
        driveToPosition(0.4,659,-671,5);
        //15. Park
        driveToPosition(0.5,-4623,-4682,5);

        //move back to park




    }




    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //negative
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(-1*Math.abs(speed));
            motorRight.setPower(-1*Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(2000);   // optional pause after each move
        }
    }
    public void driveToPosition(double speed,
                             int leftPosition, int rightPosition,
                             double timeoutS) {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            motorLeft.setTargetPosition(leftPosition);
            motorRight.setTargetPosition(rightPosition);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", leftPosition,  rightPosition);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(2000);   // optional pause after each move
        }
    }

}
