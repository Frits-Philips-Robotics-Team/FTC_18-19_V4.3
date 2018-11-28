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
    private ElapsedTime ArmServoTime = new ElapsedTime();
    private ElapsedTime BoxServoTime = new ElapsedTime();
    private DcMotor LeftDrive   = null;
    private DcMotor RightDrive  = null;
    private DcMotor Lift        = null;
    private DcMotor IntakeSpin  = null;
    private Servo ArmL          = null;
    private Servo ArmR          = null;
    private Servo Box           = null;

    // These values control the servo speed
    private final double ArmServoDelta = 0.03;
    private final double ArmServoDelayTime = 0.06;
    private double ArmTgtPos;

    private final double BoxServoDelta = 0.05;
    private final double BoxServoDelayTime = 0.03;
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
        IntakeSpin = hardwareMap.get(DcMotor.class, "IntakeSpin");
        ArmL       = hardwareMap.get(Servo.class, "ArmL");
        ArmR       = hardwareMap.get(Servo.class, "ArmR");
        Box        = hardwareMap.get(Servo.class, "Box");

        // Give the servo target position a first value
        ArmTgtPos = 1;
        BoxTgtPos = -0.7;

        // Reverse the motor that runs backwards
        LeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RightDrive.setDirection(DcMotor.Direction.FORWARD);

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
        double ArmLPos;
        double ArmRPos;

        // POV Mode uses left stick to go forward, and right stick to turn.
        double Drive = -gamepad1.left_stick_y;
        double Turn  =  gamepad1.right_stick_x;
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
            RightDrive.setPower(-(RightPower * RightPower));
        }
        else if(RightPower >= 0) {
            RightDrive.setPower(RightPower * RightPower);
        }

        // Control lift
        double LiftControl = gamepad2.left_stick_y;
        Lift.setPower(0.75 * LiftControl);

        // Control intake arm servo's slowly. First use range from -1 to 1 for easy math, then scale
        // to proper servo position
        if(ArmServoTime.time() > ArmServoDelayTime) {
            if(gamepad2.a) {
                ArmTgtPos = Range.clip(ArmTgtPos + ArmServoDelta, -1, 1);
            }
            else if(gamepad2.y) {
                ArmTgtPos = Range.clip(ArmTgtPos - ArmServoDelta, -1, 1);
            }
            ArmLPos = ((ArmTgtPos * 0.3) + 0.6);
            ArmRPos = ((-ArmTgtPos * 0.3)  + 0.4);
            ArmL.setPosition(ArmLPos);
            ArmR.setPosition(ArmRPos);
            ArmServoTime.reset();
        }
        
        IntakeSpin.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        if(BoxServoTime.time() > BoxServoDelayTime) {
            if(gamepad2.dpad_down) {
                BoxTgtPos = Range.clip(BoxTgtPos - BoxServoDelta, -1, 1);
            }
            else if(gamepad2.dpad_up) {
                BoxTgtPos = Range.clip(BoxTgtPos + BoxServoDelta, -1, 1);
            }
            Box.setPosition((BoxTgtPos * 0.325) + 0.375);
            BoxServoTime.reset();
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("ArmTgtPos", ArmTgtPos);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Servos", "left (%.2f), right (%.2f)", ArmL.getPosition(), ArmR.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}