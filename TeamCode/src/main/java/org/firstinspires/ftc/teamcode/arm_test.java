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
@TeleOp(name="Arm Test", group="Linear Opmode")
// @Disabled
public class arm_test extends LinearOpMode {

    // Timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Arm
    DcMotorEx arm;
    DcMotorEx extension;

    public static PIDFCoefficients armPidfCoefficients = new PIDFCoefficients(10,0,0,0);
    public static double ARM_POSITION_COEFF = 55;
    public static double ARM_POWER = 0.8;
    public static PIDFCoefficients extPidfCoefficients = new PIDFCoefficients(10,0,0,0);
    public static double EXT_POSITION_COEFF = 250;
    public static double EXT_POWER = 0.8;
    // Positions for arm, H= 400, M= 150, L= -200, P= -250, armpid = 10, armcoeff = 55
    // Position for extension 72, armpid=10, armcoeff = 250
    public static int HIGHEST_POS = 400;
    public static int MIDDLE_POS = 150;
    public static int LOW_POS = -150;
    public static int PICK_POS = -210;
    public static int EXT_POS = 72;


    // Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // High position of arm
            if (gamepad1.x){
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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", arm.getCurrentPosition());
            telemetry.addData("Feedforward", armPidfCoefficients.f);
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
