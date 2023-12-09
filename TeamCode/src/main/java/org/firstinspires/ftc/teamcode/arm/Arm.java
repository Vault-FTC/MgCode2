package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private DcMotorEx elbowMotor = null;

    private DcMotorEx weGoUpMotor = null;
    private Servo wristServo;
    private Servo grabberServo;

    double wristPosition = 0; //TODO : SET DEFAULT Position,
    double grabberPosition = 0;
    double targetElbowPosition = 0; // TODO : Set default.

    Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry tele)
    {
        telemetry = tele;
        elbowMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        grabberServo = hardwareMap.get(Servo.class, "grabber");
        //weGoUpMotor = hardwareMap.get(DcMotorEx.class, "extend");


        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setTargetPosition(0);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(1.0);
        elbowMotor.setPositionPIDFCoefficients(5);
        targetElbowPosition = elbowMotor.getCurrentPosition();
        grabberServo.setPosition(0.5);
        grabberPosition = 0.5;
        //elbowMotor.setTargetPositionTolerance();
    }

    public void moveArmToPosition(ArmPositions armGoals)
    {
        wristPosition = armGoals.wristPosition;
        targetElbowPosition = armGoals.elbowPosition;
        armGoals.sliderPosition = armGoals.sliderPosition;
    }

    public void moveArmByGamepad(Gamepad gamepad2)
    {
        double grabberModifier = 0;
        if(gamepad2.dpad_left)
        {
            grabberModifier = 0.002;
        }
        else if (gamepad2.dpad_right)
        {
            grabberModifier = -0.002;
        }
        grabberPosition = Math.min(1.0, Math.max(grabberPosition - (grabberModifier), 0));

        wristPosition = Math.min(1.0, Math.max(wristPosition - (0.006 * gamepad2.right_stick_y), 0));
        targetElbowPosition = Math.min(10000, Math.max(targetElbowPosition - (10 * gamepad2.left_stick_y), -10000));
    }

    public boolean checkPosition(ArmPositions target)
    {
        // WristPosition is a servo.  Servos have a range of 0.0-1.0.
        if(Math.abs(target.wristPosition - wristPosition) > 0.05)
        {
            return false;
        }
        // GrabberPosition is a servo.  Servos have a range of 0.0-1.0.
        if(Math.abs(target.grabberPosition - grabberServo.getPosition()) > 0.05)
        {
            return false;
        }
        // ElbowPosition is a motor at 1:264 reduction.  Safe range is 0-3180
        if(Math.abs(target.elbowPosition - elbowMotor.getCurrentPosition()) > 20)
        {
            return false;
        }
        if(Math.abs(target.sliderPosition - weGoUpMotor.getCurrentPosition()) > 0.10)
        {
            return false;
        }
        return true;
    }

    public void setMotors()
    {
        grabberServo.setPosition(grabberPosition);
        elbowMotor.setTargetPosition((int)targetElbowPosition);
        wristServo.setPosition(wristPosition);
        //weGoUpMotor.setPower(gamepad2.left_stick_y);

        telemetry.addData("Wrist", "%4.2f", wristPosition);
        telemetry.addData("Elbow Position", "%4.2f", targetElbowPosition);
    }
}
