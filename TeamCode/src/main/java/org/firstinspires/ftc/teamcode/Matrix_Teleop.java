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
@TeleOp(name="Matrix Driver Control", group="Teleoperation")
//@Disabled
public class Matrix_Teleop extends LinearOpMode {

    // Timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Declare robot
    HardwareMecanum robot = new HardwareMecanum();

    // Power to chassis
    public static double MOVE = 0.8;
    public static double STOP = 0;

    // PID for arm
    public static PIDFCoefficients armPidfCoefficients = new PIDFCoefficients(0,0,0,0);
    public static double ARM_POSITION_COEFF = 200;
    public static double ARM_POWER = 0.8;
    public static int HIGHEST_POS = 0;
    public static int MIDDLE_POS = 0;
    public static int LOW_POS = 0;
    public static int PICK_POS = 0;

    // Turret power
    public static double TURRET_POWER = 0.5;

    // Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        // Dashboard config
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        robot.arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armPidfCoefficients);
        robot.arm.setPositionPIDFCoefficients(ARM_POSITION_COEFF);

        // Get arm and turret ready
        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(ARM_POWER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Move Forward
            if (gamepad1.dpad_up){
                moveRobot(MOVE, MOVE, MOVE, MOVE);
            }
            // Move Reverse
            else if (gamepad1.dpad_down){
                moveRobot(-MOVE, -MOVE, -MOVE, -MOVE);
            }
            // Glide Right
            else if (gamepad1.dpad_right){
                moveRobot(MOVE, -MOVE, -MOVE, MOVE);
            }
            // Glide Left
            else if (gamepad1.dpad_left){
                moveRobot(-MOVE, MOVE, MOVE, -MOVE);
            }
            // Turn Left
            else if (gamepad1.left_stick_x < 0){
                moveRobot(-MOVE, -MOVE, MOVE, MOVE);
            }
            // Turn Right
            else if (gamepad1.left_stick_x > 0){
                moveRobot(MOVE, MOVE, -MOVE, -MOVE);
            }
            // Arm Top
            else if (gamepad1.y){
                moveArm(HIGHEST_POS);
            }
            // Arm Middle
            else if (gamepad1.x){
                moveArm(MIDDLE_POS);
            }
            // Arm Bottom
            else if (gamepad1.a){
                moveArm(LOW_POS);
            }
            // Arm Pick
            else if (gamepad1.b){
                moveArm(PICK_POS);
            }
            // Take object in
            else if (gamepad1.right_bumper){
                robot.fan.setPower(1);
            }
            // Push object out
            else if (gamepad1.left_bumper){
                robot.fan.setPower(-1);
            }
            // Move Turret
            else if (gamepad1.left_stick_y < 0){
                robot.turret.setPower(TURRET_POWER);
            }
            else if (gamepad1.left_stick_y > 0){
                robot.turret.setPower(-TURRET_POWER);
            }
            // Move carousel
            else if (gamepad1.right_trigger > 0){
                robot.carousel.setPower(1);
            }
            else if (gamepad1.left_trigger < 0){
                robot.carousel.setPower(-1);
            }
            // Stop robot
            else{
                moveRobot(STOP, STOP, STOP, STOP);
                robot.fan.setPower(STOP);
                robot.turret.setPower(STOP);
                robot.carousel.setPower(STOP);
            }
            // Telemetry data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
            telemetry.addData("Extension Position", robot.turret.getCurrentPosition());
            telemetry.update();
        }
    }
    public void moveArm(int targetPos){
        if(robot.arm.getCurrentPosition() > targetPos){
            armPidfCoefficients.f = -5;
        }
        else if(robot.arm.getCurrentPosition() < targetPos){
            armPidfCoefficients.f = 0;
        }
        robot.arm.setTargetPosition(targetPos);
        robot.arm.setPower(ARM_POWER);
    }
    public void moveRobot(double leftForward, double leftBackward, double rightForward, double rightBackward){
        robot.leftForward.setPower(leftForward);
        robot.leftBack.setPower(leftBackward);
        robot.rightForward.setPower(rightForward);
        robot.rightBack.setPower(rightBackward);
    }
}
