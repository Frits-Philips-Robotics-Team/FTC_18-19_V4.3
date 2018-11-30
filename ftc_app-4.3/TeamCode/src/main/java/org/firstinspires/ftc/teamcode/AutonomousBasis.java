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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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

@Autonomous(name="AutonomousBasis", group="Pushbot")
//@Disabled
public class AutonomousBasis extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime ArmServoTime = new ElapsedTime();
    private ElapsedTime BoxServoTime = new ElapsedTime();
    private DcMotor LeftDrive   = null;
    private DcMotor RightDrive  = null;
    private DcMotor Lift        = null;
    private DcMotor IntakeSpin  = null;
    private Servo ArmL          = null;
    private Servo ArmR          = null;
    private Servo Box           = null;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYrJcPz/////AAABmeoL9kpl/ELOsQ43TkTjFAxba3UVdN2Xo71qDtfCBkTKYPkUZOWTbJ3AgYW0HtpPZ1pecxwUjMYiN4BtLt32s097m/E+/LUSLc6waPrJe/fnSekZxd7WUkU8Fb/f6CZLthxCrKt5nzdCx2LLg3Sfjpegd29NVZDhG/oZYD6wYp28jFqqsHzJ6D1v8RcoRYmdXPpNHFjU0dW4pj2i+wwdVZtWRDixBIb+79fnNT3EQf9E897pOf5Ea30Td1uDwotmvt78uKUe7Hi1Z/7pfBtLCgemUJSlLuA5Fviuesy3kTifpwLH3m1pIcTo8tzwwc7NPa8YG9PFsivWf6+wyW2NOlT9JwaBWvgbs6oy8kdsoc+x";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String GoldPos;
    private boolean isGoldKnocked = false;

    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // REV HD hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 0.66666667 ;
    static final double     WHEEL_DIAMETER_CM       = 10.16 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_CM * 3.1415);

    static final double     LIFT_COUNTS_PER_CM      = 509.31083877 ;    // 2240 / (1.4 * 3.1415)

    @Override
    public void runOpMode() {
          //Initialize the drive system variables.
        LeftDrive  = hardwareMap.get(DcMotor.class, "MotorL");
        RightDrive = hardwareMap.get(DcMotor.class, "MotorR");
        Lift       = hardwareMap.get(DcMotor.class, "Lift");
        IntakeSpin = hardwareMap.get(DcMotor.class, "IntakeSpin");
        ArmL       = hardwareMap.get(Servo.class, "ArmL");
        ArmR       = hardwareMap.get(Servo.class, "ArmR");
        Box        = hardwareMap.get(Servo.class, "Box");

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);

        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          LeftDrive.getCurrentPosition(),
                          RightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        ArmL.setPosition(0.6);
//        ArmR.setPosition(0.4);
//        sleep(500);
//        Box.setPosition(0.4);
//        sleep(1000);
        //EncoderLift(0.5, -8, 3);
       // encoderDrive(0.75, 5, 5, 1);
        //EncoderLift(0.5, 16, 3);
        if(opModeIsActive()) {
            if(tfod != null) {
                tfod.activate();
            }
            while(!isGoldKnocked && opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if(updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 1) {
                            for (Recognition recognition : updatedRecognitions) {
                                if(GoldPos == null) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        GoldPos = "center";
                                        telemetry.addData("GoldPos: ", "center");
                                    }
                                    else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        encoderDrive(0.75, 8, 8, 2);
                                        encoderDrive(0.75, -7, 7, 2);
                                        encoderDrive(0.75, 8, 8, 2);
                                        GoldPos = "notCenter";
                                        telemetry.addData("GoldPos: ", "NotCenter");
                                    }
                                }
                                else if(GoldPos.equals("notCenter")) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        GoldPos = "left";
                                        telemetry.addData("GoldPos: ", "Left");
                                    }
                                    else if(recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        GoldPos = "right";
                                        telemetry.addData("GoldPos: ", "Right");
                                    }
                                }
                            }
                        }
                        telemetry.update();
                    }

                }
                if(GoldPos != null) {
                    if(GoldPos.equals("left")) {
                        encoderDrive(0.75, 60, 60, 5);
                        encoderDrive(0.75, 12, -12, 2);
                        encoderDrive(0.75, 20, 20, 3);
                        isGoldKnocked = true;
                    }
                    else if(GoldPos.equals("right")) {
                        encoderDrive(0.75, -8, -8, 2);
                        encoderDrive(0.75, 14, -14, 2);
                        encoderDrive(0.75, 60, 60,5);
                        encoderDrive(0.75, -12, 12, 2);
                        encoderDrive(0.75, 20, 20, 3);
                        isGoldKnocked = true;
                    }
                    else if(GoldPos.equals("center")) {
                        encoderDrive(0.75, 75, 75, 6);
                        isGoldKnocked = true;
                    }
                }

            }
        }
        if(tfod != null) {
            tfod.shutdown();
        }

        //sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftCM, double rightCM,
                             double TimeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LeftDrive.getCurrentPosition() + (int) (leftCM * COUNTS_PER_CM);
            newRightTarget = RightDrive.getCurrentPosition() + (int) (rightCM * COUNTS_PER_CM);
            LeftDrive.setTargetPosition(newLeftTarget);
            RightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftDrive.setPower(Math.abs(speed));
            RightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < TimeoutS) &&
                    (LeftDrive.isBusy() && RightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        LeftDrive.getCurrentPosition(),
                        RightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftDrive.setPower(0);
            RightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void EncoderLift(double speed, double cm, double TimeoutS) {
        int NewLiftTarget;

        if(opModeIsActive()) {
            NewLiftTarget = Lift.getCurrentPosition() + (int) (cm * LIFT_COUNTS_PER_CM);
            Lift.setTargetPosition(NewLiftTarget);

            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            Lift.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < TimeoutS) && Lift.isBusy()) {
                telemetry.addData("Path1", "Lifting to %7d", NewLiftTarget);
                telemetry.addData("Path2", "Lift now at %7d", Lift.getCurrentPosition());
                telemetry.update();
            }

            Lift.setPower(0);

            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
