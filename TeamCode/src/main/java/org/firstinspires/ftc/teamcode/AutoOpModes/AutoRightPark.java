package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;

@Autonomous (name= "AutoRightPark")
public class AutoRightPark extends LinearOpMode {
    RobotMecanumDrive drive;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new RobotMecanumDrive(hardwareMap, telemetry);
        drive.drive(false, 0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = 0;
            double lateral = 0;
            double yaw = 0;

            if (runtime.seconds() < 0.2) // 2'
            {
                drive.drive(false, .5, 0, 0);
            } else if (runtime.seconds() < 0.6) {
                drive.drive(false,0,0,0);
            } else if (runtime.seconds() < 1.7) {
                drive.drive(false, 0, .8, 0);
            } else{
                drive.drive(false, 0,0,0);
            }

        }
    }
}

