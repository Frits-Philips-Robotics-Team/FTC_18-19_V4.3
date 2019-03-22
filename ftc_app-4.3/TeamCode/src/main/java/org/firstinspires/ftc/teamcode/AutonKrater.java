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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.sql.Driver;
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

@Autonomous(name="AutonKrater", group="Linear Opmode")
//@Disabled
public class AutonKrater extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime ArmServoTime = new ElapsedTime();
    private ElapsedTime BoxServoTime = new ElapsedTime();
    private DcMotor LeftDrive   = null;
    private DcMotor RightDrive  = null;
    private DcMotor Lift        = null;
    private DcMotorSimple IntakeSpin = null;
    private DcMotorSimple ArmL  = null;
    private DcMotor ArmR        = null;
    private Servo Hook          = null;
    private Servo BoxL          = null;
    private Servo BoxR          = null;

    BNO055IMU imu;
    Orientation angles;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYrJcPz/////AAABmeoL9kpl/ELOsQ43TkTjFAxba3UVdN2Xo71qDtfCBkTKYPkUZOWTbJ3AgYW0HtpPZ1pecxwUjMYiN4BtLt32s097m/E+/LUSLc6waPrJe/fnSekZxd7WUkU8Fb/f6CZLthxCrKt5nzdCx2LLg3Sfjpegd29NVZDhG/oZYD6wYp28jFqqsHzJ6D1v8RcoRYmdXPpNHFjU0dW4pj2i+wwdVZtWRDixBIb+79fnNT3EQf9E897pOf5Ea30Td1uDwotmvt78uKUe7Hi1Z/7pfBtLCgemUJSlLuA5Fviuesy3kTifpwLH3m1pIcTo8tzwwc7NPa8YG9PFsivWf6+wyW2NOlT9JwaBWvgbs6oy8kdsoc+x";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String GoldPos;
    private boolean isGoldKnocked = false;

    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // REV HD hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_CM       = 10.16 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_CM * 3.1415);

    static final double     LIFT_COUNTS_PER_CM      = 28.8 ; //288 / 10 cm

    static final double     DRIVE_SPEED             = 0.6 ;
    static final double     TURN_SPEED              = 0.4;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

          //Initialize the system variables.
        LeftDrive  = hardwareMap.get(DcMotor.class, "MotorL");
        RightDrive = hardwareMap.get(DcMotor.class, "MotorR");
        Lift       = hardwareMap.get(DcMotor.class, "Lift");
        IntakeSpin = hardwareMap.get(DcMotorSimple.class, "IntakeSpin");
        ArmL       = hardwareMap.get(DcMotorSimple.class, "ArmL");
        ArmR       = hardwareMap.get(DcMotor.class, "ArmR");
        Hook       = hardwareMap.get(Servo.class, "Hook");
        BoxL       = hardwareMap.get(Servo.class, "BoxL");
        BoxR       = hardwareMap.get(Servo.class, "BoxR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


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
        ArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          LeftDrive.getCurrentPosition(),
                          RightDrive.getCurrentPosition());
        telemetry.update();

        Hook.setPosition(0.8);
        //Lift.setPower(0.1); // Keeps the robot hanging
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        ArmR.setPower(1);
        ArmR.setTargetPosition(70);
        while(ArmR.isBusy() && opModeIsActive());
        ArmR.setPower(0);
        ArmL.setPower(0);
        BoxL.setPosition(0.5);
        BoxR.setPosition(0.28);
        sleep(400);
        Lift.setPower(0.7);
        Hook.setPosition(0.4);
        sleep(800);
        EncoderLift(-1, -27, 5);
        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, 2, 2, 1);
        EncoderLift(1, 25, 5);
        ArmR.setPower(-0.8);
        ArmL.setPower(-0.6);
        ArmR.setTargetPosition(20);
        while(ArmR.isBusy() && opModeIsActive());
        ArmR.setPower(0);
        ArmL.setPower(0);
        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, 10, 10, 2);
        encoderDrive(TURN_SPEED, TURN_SPEED, 3,-5, 1);
       //encoderDrive(DRIVE_SPEED, 6, 6, 2);
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
                                    if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        GoldPos = "notRight";
                                        telemetry.addData("GoldPos: ", "notRight");
                                        //encoderDrive(DRIVE_SPEED, -8, -8, 1);
                                        encoderDrive(TURN_SPEED, TURN_SPEED,-6, 9, 1);
                                        //encoderDrive(DRIVE_SPEED, 3, 3, 1);
                                    }
                                    else if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        GoldPos = "right";
                                        telemetry.addData("GoldPos: ", "right");
                                    }
                                }
                                else if(GoldPos.equals("notRight")) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        GoldPos = "center";
                                        telemetry.addData("GoldPos: ", "center");
                                    }
                                    else if(recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        GoldPos = "left";
                                        telemetry.addData("GoldPos: ", "left");
                                    }
                                }
                            }
                        }
                        telemetry.update();
                    }

                }
                if(GoldPos != null) {
                    if(GoldPos.equals("left")) {
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED,-10, -10, 2);
                        encoderDrive(TURN_SPEED, TURN_SPEED,-3, 6, 2);
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED,30, 30, 2);
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, -18, -18, 1);
                        encoderDrive(TURN_SPEED, TURN_SPEED, 30, -30, 2);
                        encoderDrive(DRIVE_SPEED, 0.7 * DRIVE_SPEED, -64, -45, 4);
                        encoderDrive(TURN_SPEED, TURN_SPEED, -15, 0, 2);
                        //encoderDrive(TURN_SPEED, TURN_SPEED, -4, -4, 2);
                        isGoldKnocked = true;
                    }
                    else if(GoldPos.equals("right")) {
                        encoderDrive(TURN_SPEED, TURN_SPEED,3, -3, 1);
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED,24, 24,2);
                        encoderDrive(TURN_SPEED, TURN_SPEED,0, -27, 2);
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, -72, -72, 4);
                        encoderDrive(TURN_SPEED, TURN_SPEED, -8, 15, 2);
                        isGoldKnocked = true;
                    }
                    else if(GoldPos.equals("center")) {
                        //encoderDrive(DRIVE_SPEED, -4,   -4, 1);
                        encoderDrive(TURN_SPEED, TURN_SPEED,5, -3, 1);
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED,18, 18, 2);
                        encoderDrive(TURN_SPEED, TURN_SPEED,0, -41, 2);
                        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, -58, -58, 3);
                        encoderDrive(TURN_SPEED, TURN_SPEED, -9, 16, 2);
                        isGoldKnocked = true;
                    }
                }

            }
        }
        if(tfod != null) {
            tfod.shutdown();
        }
        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, 20,20, 1);
        ArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmR.setPower(0.7);
        ArmL.setPower(0.7);
        sleep(500);
        ArmR.setPower(0);
        ArmL.setPower(0);
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
    public void encoderDrive(double leftSpeed, double rightSpeed,
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
            LeftDrive.setPower(Math.abs(leftSpeed));
            RightDrive.setPower(Math.abs(rightSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < TimeoutS) &&
                    (LeftDrive.isBusy() || RightDrive.isBusy())) {

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

            sleep(100);   // optional pause after each move
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

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        LeftDrive.setPower(leftSpeed);
        RightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
