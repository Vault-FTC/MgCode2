package org.firstinspires.ftc.teamcode.AutoOpModes;

import org.firstinspires.ftc.teamcode.AutoOpModes.SimpleOpenCVColorOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;


@Autonomous(name="SimpleParkAuto")
public class SimpleParkAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    RobotMecanumDrive drive = new RobotMecanumDrive(hardwareMap, telemetry);
    public static double DISTANCE = 50;
    /*
        private Encoder left = null;

        private Encoder right = null;

        private Encoder back = null;
        */
    @Override
    public void runOpMode() {



        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        double leftFrontPower = 1;
        double rightFrontPower = 1;
        double leftBackPower = 1;
        double rightBackPower = 1;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Boolean getBlue = (boolean) SimpleOpenCVColorOpMode.getBlue;

        if (getBlue = true){
        drive.followTrajectory(trajectoryForward);
        }

            Boolean getRed = (boolean) SimpleOpenCVColorOpMode.getRed;

        if (getRed = true){}




        //============================== test
//        leftFrontDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        leftBackDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightFrontDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightBackDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //================================

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
    }
}
