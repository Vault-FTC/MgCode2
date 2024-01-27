package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LinearSlidw.LinearSlide;

import java.util.LinkedList;
import java.util.List;

public class Arm {

    private DcMotorEx elbowMotor = null;

    private Servo wristServo;
    private DigitalChannel limitSwitch = null;

    double wristPosition = 0; //TODO : SET DEFAULT Position,
    double sliderPosition = 0;
    double targetElbowPosition = 0; // TODO : Set default.
    double targetSliderPosition = 0; // TODO : I think same as before

    ArmPositions targetPosition = null;

    double lastNewTimer = 0;

    private final double ARM_ELBOW_HIGH_SPEED = 3;

    private final double ARM_MIN_POSITION = -6000;

    private final double ARM_MAX_POSITION = 3028;

    private boolean isInitialized = false;


    Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry tele)
    {
        telemetry = tele;
        elbowMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "arm_limit");

        wristServo = hardwareMap.get(Servo.class, "wrist");
        wristServo.setPosition(1.0);
        wristPosition = 1.0;
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setTargetPosition(0);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(1.0);
        elbowMotor.setPositionPIDFCoefficients(ARM_ELBOW_HIGH_SPEED);
        targetElbowPosition = elbowMotor.getCurrentPosition();
    }

    public void initServo()
    {

    }

    public void initializeArm(ElapsedTime runtime)
    {
        if(isInitialized)
        {
            return;
        }
        if(lastNewTimer == 0)
        {
            runtime.reset();
            lastNewTimer = runtime.seconds();
            elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbowMotor.setPower(-0.3);
        }
        if(Math.abs( runtime.seconds() - lastNewTimer) > 10)
        {
            isInitialized = true;
            elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbowMotor.setTargetPosition(0);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(1.0);
            elbowMotor.setPositionPIDFCoefficients(ARM_ELBOW_HIGH_SPEED);
            targetElbowPosition = elbowMotor.getCurrentPosition();
            return;
        }
        telemetry.addData("IsInitializing = ", true);
        if(limitSwitch.getState())
        {
            isInitialized = true;
            elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbowMotor.setTargetPosition(0);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(1.0);
            elbowMotor.setPositionPIDFCoefficients(ARM_ELBOW_HIGH_SPEED);
            targetElbowPosition = elbowMotor.getCurrentPosition();
        }
        telemetry.addData("IsInitialized : ", isInitialized);
    }


    public void moveArmToPosition(ArmPositions armGoals)
    {
        wristPosition = armGoals.wristPosition;
        targetElbowPosition = armGoals.elbowPosition;
        sliderPosition = armGoals.sliderPosition;

        elbowMotor.setPositionPIDFCoefficients(1.3);
    }

    public void moveArmByGamepad(Gamepad gamepad2)
    {
        wristPosition = Math.min(1.0, Math.max(wristPosition - (0.006 * gamepad2.right_stick_y), 0));
        targetElbowPosition = Math.min(ARM_MAX_POSITION, Math.max(targetElbowPosition - (10 * gamepad2.left_stick_y), ARM_MIN_POSITION));
    }

    public boolean checkPosition(ArmPositions target)
    {
        // WristPosition is a servo.  Servos have a range of 0.0-1.0.
//        if(Math.abs(target.wristPosition - wristPosition) > 0.05)
//        {
//            return false;
//        }
        // ElbowPosition is a motor at 1:264 reduction.  Safe range is 0-3180
        if(Math.abs(target.elbowPosition - elbowMotor.getCurrentPosition()) > 20)
        {
            return false;
        }
        if(Math.abs(target.sliderPosition - LinearSlide.WeGoUp.getCurrentPosition()) > 120)
        {
            return false;
        }
        return true;
    }

    public ArmPhases addNextArmTargets(List<ArmPositions> armTargets, ArmPhases currentPhase) {
        ArmPhases nextPhase = currentPhase;
        if (armTargets.size() < 1) {
            switch (currentPhase) {
                case ARM_INITIALIZATION:
                        //set pos to 0
                        ArmPositions.BuildArmStartupToReadyPickup(armTargets);
                        nextPhase = ArmPhases.ARM_READYPICKUP_TO_PICKUP;
                        break;
                case ARM_READYPICKUP_TO_PICKUP:
                    ArmPositions.BuildReadyPickupToPickup(armTargets);
                    elbowMotor.setPositionPIDFCoefficients(0.8);
                    nextPhase = ArmPhases.ARM_PICKUP_TO_PREPARETOPLACE;
                    break;
                case ARM_PICKUP_TO_PREPARETOPLACE:
                    ArmPositions.BuildPickupToPrepareToPlace(armTargets);
                    elbowMotor.setPositionPIDFCoefficients(1.3);
                    nextPhase = ArmPhases.ARM_INITIALIZATION;
                    break;
                case ARM_PREPARETOPLACE_TO_READYPICKUP:
                    ArmPositions.BuildPrepareToPlaceToReadyToPickup(armTargets);
                    nextPhase = ArmPhases.ARM_INITIALIZATION;
                    break;
            }
        }
        return nextPhase;
    }

    public void doArm(Gamepad gamepad2, LinkedList<ArmPositions> armTargets, ElapsedTime runtime, LinearSlide slider)
    {
        telemetry.addData("Arm Limit : ", limitSwitch.getState());

        if(gamepad2.start)
        {
            armTargets.clear();
            targetPosition = null;
        }
        if(armTargets.size() > 0 && targetPosition == null)
        {
            targetPosition = armTargets.getFirst();
            armTargets.removeFirst();
            slider.MoveSliderToPosition(targetPosition, runtime);
            moveArmToPosition(targetPosition);
            lastNewTimer = runtime.seconds();
        }
        boolean hasReachedTarget = true;
        if(targetPosition == null)
        {
            moveArmByGamepad(gamepad2);
        }
        else {
            hasReachedTarget = checkPosition(targetPosition);
            telemetry.addData("HasReachedTarget: ", hasReachedTarget);
            telemetry.addData("Slider HasReachedTarget: ", targetPosition.sliderPosition - LinearSlide.WeGoUp.getCurrentPosition());
            telemetry.addData("Slider Target Position", targetPosition.sliderPosition);
            telemetry.addData( "Target Pos Elbow: ", targetPosition.elbowPosition + "\nTarget Pos Wrist: " + wristPosition);
            telemetry.addData("LastNewTimer :", lastNewTimer);
            if (hasReachedTarget) {
                if(runtime.seconds() - lastNewTimer > .5) {
                    targetPosition = null;
                    if(armTargets.size() == 0)
                    {
                        elbowMotor.setPositionPIDFCoefficients(ARM_ELBOW_HIGH_SPEED);
                    }
                }
            }
        }
        setMotors();
        telemetry.addData("Arm Targets Size: ", armTargets.size());
        if(armTargets.size() > 0) {
            //telemetry.addData("Wrist :", "%4.2d, %4.2d", wristPosition, armTargets.get(0).wristPosition);
            //telemetry.addData("Elbow :", "%4.2d, %4.2d", elbowMotor.getCurrentPosition(), targetElbowPosition);
        }
    }

    public void setMotors()
    {
        elbowMotor.setTargetPosition((int)targetElbowPosition);
        wristServo.setPosition(wristPosition);

        telemetry.addData("Wrist", "%4.2f", wristPosition);
        telemetry.addData("Elbow Position", "%4.2f", targetElbowPosition);
    }
}
