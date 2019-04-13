package org.firstinspires.ftc.teamcode;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RijdenV3", group="Iterative Opmode")
//@Disabled
public class RijdenV3 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime BoxServoTime = new ElapsedTime();
    private DcMotor LeftDrive   = null;
    private DcMotor RightDrive  = null;
    private DcMotorSimple IntakeSpin  = null;
    private DcMotor Lift        = null;
    private DcMotorSimple ArmL  = null;
    private DcMotor ArmR        = null;
    private Servo BoxL          = null;
    private Servo BoxR          = null;
    private Servo Hook          = null;

    // These values control the servo speed
    private final double BoxServoDelta = 0.07;
    private final double BoxServoDelayTime = 0.02;
    private double BoxTgtPos;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables
        LeftDrive  = hardwareMap.get(DcMotor.class, "MotorL");
        RightDrive = hardwareMap.get(DcMotor.class, "MotorR");
        Lift       = hardwareMap.get(DcMotor.class, "Lift");
        IntakeSpin = hardwareMap.get(DcMotorSimple.class, "IntakeSpin");
        ArmL       = hardwareMap.get(DcMotorSimple.class, "ArmL");
        ArmR       = hardwareMap.get(DcMotor.class, "ArmR");
        BoxL       = hardwareMap.get(Servo.class, "BoxL");
        BoxR       = hardwareMap.get(Servo.class, "BoxR");
        Hook       = hardwareMap.get(Servo.class, "Hook");

        // Give the servo target position a first value
        BoxTgtPos = 0.6;

        // Reverse the motor that runs backwards
        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double LeftPower;
        double RightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        double Drive = 1 * gamepad1.left_stick_y;
        double Turn  = -1 * gamepad1.right_stick_x;
        LeftPower    = Range.clip(Drive + Turn, -1.0, 1.0) ;
        RightPower   = Range.clip(Drive - Turn, -1.0, 1.0) ;

        // Send calculated power to wheels in an exponential function for better precision
        if(LeftPower < 0) {
            LeftDrive.setPower(-(LeftPower * LeftPower));
        }
        else if(LeftPower >= 0) {
            LeftDrive.setPower(LeftPower * LeftPower);
        }
        if(RightPower < 0) {
            RightDrive.setPower(RightPower * RightPower);
        }
        else if(RightPower >= 0) {
            RightDrive.setPower(-(RightPower * RightPower));
        }

        // Control lift
        double LiftControl = gamepad2.left_stick_y;
        Lift.setPower(LiftControl);

        ArmL.setPower(0.8 * (gamepad1.left_trigger - gamepad1.right_trigger));
        ArmR.setPower(0.8 * (gamepad1.left_trigger - gamepad1.right_trigger));
        IntakeSpin.setPower(0.85 * (gamepad2.left_trigger - gamepad2.right_trigger));

        if(BoxServoTime.time() > BoxServoDelayTime) {
            if(gamepad2.dpad_down) {
                BoxTgtPos = Range.clip(BoxTgtPos + BoxServoDelta, -1, 0.6);
            }
            else if(gamepad2.dpad_up) {
                BoxTgtPos = Range.clip(BoxTgtPos - BoxServoDelta, -1, 0.6);
            }
            //Box.setPosition((BoxTgtPos * 0.325) + 0.375);
            BoxL.setPosition((BoxTgtPos * 0.3625) + 0.3625);
            BoxR.setPosition((BoxTgtPos * -0.3625) + 0.47);
            BoxServoTime.reset();
        }

        if(gamepad2.dpad_left) {
            Hook.setPosition(0.4);
        }
        else if(gamepad2.dpad_right) {
            Hook.setPosition(0.8);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("BoxTgtPos", BoxTgtPos);
        telemetry.addData("LiftPower", Lift.getPower());
        telemetry.addData("ArmPower", ArmL.getPower());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
