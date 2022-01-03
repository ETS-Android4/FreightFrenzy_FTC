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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="SB Driver Control", group="Teleoperation")
//@Disabled
public class SB_Teleop extends LinearOpMode {

    // Timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Chassis
    DcMotorEx leftDrive;
    DcMotorEx rightDrive;
    public static double MOVE = 0.8;
    public static double TURN = 0.8;

    // Arm and Extension
    DcMotorEx arm;
    DcMotorEx extension;
    Servo leftClaw;
    Servo rightClaw;

    public static PIDFCoefficients armPidfCoefficients = new PIDFCoefficients(0,0,0,0);
    public static PIDFCoefficients extPidfCoefficients = new PIDFCoefficients(0,0,0,0);
    public static double ARM_POSITION_COEFF = 200;
    public static double EXT_POSITION_COEFF = 200;
    public static double ARM_POWER = 0.8;
    public static double EXT_POWER = 0.8;

    public static int HIGHEST_POS = 0;
    public static int MIDDLE_POS = 0;
    public static int LOW_POS = 0;
    public static int PICK_POS = 0;
    public static int EXT_POS = 0;

    public static double GRIP = 0.85;
    public static double DROP = 0.6;

    // Carousel
    DcMotorEx carousel;

    // Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        // Dashboard config
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Chassis config
        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm config
        arm  = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armPidfCoefficients);
        arm.setPositionPIDFCoefficients(ARM_POSITION_COEFF);

        // Extension config
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setDirection(DcMotorSimple.Direction.REVERSE);

        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, extPidfCoefficients);
        extension.setPositionPIDFCoefficients(EXT_POSITION_COEFF);

        // Claw config
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        // Carousel config
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        // Get arm and extension ready
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
        extension.setTargetPosition(0);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(EXT_POWER);

        arm.setTargetPosition(150);
        arm.setPower(ARM_POWER);
        sleep(1000);
        extension.setTargetPosition(EXT_POS);
        extension.setPower(EXT_POWER);

        // Get claw ready
        leftClaw.setPosition(0.6);
        rightClaw.setPosition(0.6);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Move forward
            if (gamepad1.dpad_up){
                leftDrive.setPower(MOVE);
                rightDrive.setPower(MOVE);
            }
            // Move backward
            else if (gamepad1.dpad_down){
                leftDrive.setPower(-MOVE);
                rightDrive.setPower(-MOVE);
            }
            // Turn left
            else if (gamepad1.dpad_left){
                leftDrive.setPower(-TURN);
                rightDrive.setPower(TURN);
            }
            // Turn right
            else if (gamepad1.dpad_right){
                leftDrive.setPower(TURN);
                rightDrive.setPower(-TURN);
            }
            // Highest position of arm
            else if (gamepad1.x){
                moveArm(HIGHEST_POS);
            }
            // Middle position of arm
            else if (gamepad1.y){
                moveArm(MIDDLE_POS);
            }
            // Low position of arm
            else if (gamepad1.b){
                moveArm(LOW_POS);
            }
            // Pick position of arm
            else if (gamepad1.a){
                moveArm(PICK_POS);
            }
            // Grip object
            else if (gamepad1.right_bumper){
                leftClaw.setPosition(GRIP);
                rightClaw.setPosition(GRIP);
            }
            // Drop object
            else if (gamepad1.left_bumper){
                leftClaw.setPosition(DROP);
                rightClaw.setPosition(DROP);
            }
            // Carousel rotate
            else if (gamepad1.left_stick_y > 0 && gamepad1.left_stick_y < 0){
                carousel.setPower(-gamepad1.left_stick_y);
            }
            // Set powers to zero to stop
            else{
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                carousel.setPower(0.0);
            }

            // Telemetry data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Extension Position", extension.getCurrentPosition());
            telemetry.update();
        }
    }
    public void moveArm(int targetPos){
        if(arm.getCurrentPosition() > targetPos){
            armPidfCoefficients.f = -5;
        }
        else if(arm.getCurrentPosition() < targetPos){
            armPidfCoefficients.f = 0;
        }
        arm.setTargetPosition(targetPos);
        arm.setPower(ARM_POWER);
    }
}
