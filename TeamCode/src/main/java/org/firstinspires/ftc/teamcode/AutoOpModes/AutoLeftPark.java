package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;

@Autonomous (name= "AutoLeftPark")
public class AutoLeftPark extends LinearOpMode {
    RobotMecanumDrive drive;
    public static double ForwardDistance = 20;
    public static double Strafe = 15;
    public static double BackwardsDist = 1;

    boolean hasRunMovement = false;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new RobotMecanumDrive(hardwareMap, telemetry);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(ForwardDistance)
                .build();
        waitForStart();
        Trajectory trajectoryBackwards = drive.trajectoryBuilder(new Pose2d())
                .forward(BackwardsDist)
                .build();
        waitForStart();
        Trajectory trajectoryStrafe = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(Strafe)
                .build();
        waitForStart();

            if (!hasRunMovement) {
                drive.followTrajectory(trajectoryForward);
                //wait(1000);
                drive.followTrajectory(trajectoryBackwards);
                //wait(1000);
                drive.followTrajectory(trajectoryStrafe);
            hasRunMovement = true;
            }

        }

    }

