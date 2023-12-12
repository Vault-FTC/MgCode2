/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmPhases;
import org.firstinspires.ftc.teamcode.arm.ArmPositions;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;
import org.firstinspires.ftc.teamcode.droneLauncher.DroneLauncher;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeModes;

import java.util.ArrayList;
import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Lucas Omni Test", group="Linear OpMode")
public class Lucas_OmniTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RobotMecanumDrive drive = new RobotMecanumDrive(hardwareMap, telemetry);
        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap, telemetry);
        Arm arm = new Arm(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        List<ArmPositions> armTargets = new ArrayList<>();
        armTargets.add(new ArmPositions(0,0,0, false));
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean fastMode = gamepad1.x;

            if (gamepad1.a) {
                intake.runIntake(IntakeModes.INTAKE_ON);
            } else if (gamepad1.b) {
                intake.runIntake(IntakeModes.INTAKE_OUT);
            } else {
                intake.runIntake(IntakeModes.INTAKE_OFF);
            }


            droneLauncher.Launch(gamepad1.y);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            drive.drive(fastMode, axial, lateral, yaw);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private ArmPhases doArm(Arm arm, List<ArmPositions> armTargets, ArmPhases currentPhase)
    {
        ArmPhases nextPhase = ArmPhases.ARM_PREPARETOPLACE_TO_READYPICKUP;
        if(armTargets.size() < 1) {
            if (gamepad2.dpad_up) {
                switch(currentPhase)
                {
                    case ARM_INITIALIZATION:
                        nextPhase = ArmPhases.ARM_STARTUP_TO_READYPICKUP;
                        break;
                    case ARM_STARTUP_TO_READYPICKUP:
                        armTargets = ArmPositions.BuildArmStartupToReadyPickup();
                        nextPhase = ArmPhases.ARM_READYPICKUP_TO_PICKUP;
                        break;
                    case ARM_READYPICKUP_TO_PICKUP:
                        armTargets = ArmPositions.BuildReadyPickupToPickup();
                        nextPhase = ArmPhases.ARM_PICKUP_TO_PREPARETOPLACE;
                        break;
                    case ARM_PICKUP_TO_PREPARETOPLACE:
                        armTargets = ArmPositions.BuildPickupToPrepareToPlace();
                        nextPhase = ArmPhases.ARM_PREPARETOPLACE_TO_READYPICKUP;
                        break;
                    case ARM_PREPARETOPLACE_TO_READYPICKUP:
                        armTargets = ArmPositions.BuildPrepareToPlaceToReadyToPickup();
                        nextPhase = ArmPhases.ARM_READYPICKUP_TO_PICKUP;
                        break;
                }
            }
        }
        if(gamepad2.start)
        {
            armTargets.clear();
        }
        if (armTargets.size() > 0) {
            if (arm.checkPosition(armTargets.get(0))) {
                if(runtime.seconds() > 1.5) {
                    armTargets.remove(0);
                }
            } else {
                arm.moveArmToPosition(armTargets.get(0));
                runtime.reset();
            }
        }
        else {
            arm.moveArmByGamepad(gamepad2);
            arm.setMotors();
        }
        return nextPhase;
    }
}
