package org.firstinspires.ftc.teamcode.LinearSlidw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.ArmPositions;

import java.util.LinkedList;


public class LinearSlide {
    private DcMotorEx WeGoUp = null;
    private final double SLIDER_MIN_POSITION = -6000;

    private final double SLIDER_MAX_POSITION = 3028;
    Telemetry telemetry;

    private final double LINEAR_SLIDE_SPEED = 3;

    private double targetSlidePosition = 0;

    private double lastNewTimer = 0;

    private ArmPositions targetPosition;

    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        WeGoUp = hardwareMap.get(DcMotorEx.class, "WeGoUp_motor");
        WeGoUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WeGoUp.setTargetPosition(0);
        WeGoUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WeGoUp.setPower(1.0);
        WeGoUp.setPositionPIDFCoefficients(LINEAR_SLIDE_SPEED);
        targetSlidePosition = WeGoUp.getCurrentPosition();

    }

    public void RunSlideMode(LinearSlideModes slideModes) {
        double linearSlideMultiplier = 0;
        switch(slideModes)
        {
            case SLIDE_OFF:
                linearSlideMultiplier = 0;
                break;
            case SLIDE_DOWN:
                linearSlideMultiplier = -30;
                break;
            case SLIDE_UP:
                linearSlideMultiplier = 30;
                break;

        }
        targetSlidePosition = Math.min(SLIDER_MAX_POSITION, Math.max(targetSlidePosition + (linearSlideMultiplier), SLIDER_MIN_POSITION));
        telemetry.addData("Slider Position", WeGoUp.getCurrentPosition());
    }

    public void MoveSliderToPosition(ArmPositions positions)
    {
        targetSlidePosition = positions.sliderPosition;
    }

    public boolean checkPosition(ArmPositions target) {
        if(Math.abs(target.sliderPosition - WeGoUp.getCurrentPosition()) > 20)
        {
            return false;
        }
        return true;
    }

    public void doSlider(Gamepad gamepad2, LinkedList<ArmPositions> armTargets, ElapsedTime runtime)
    {
        if(gamepad2.start)
        {
            armTargets.clear();
        }
        if(armTargets.size() > 0 && targetPosition == null)
        {
            targetPosition = armTargets.getFirst();
            armTargets.removeFirst();
            MoveSliderToPosition(targetPosition);
            lastNewTimer = runtime.seconds();
        }

        boolean hasReachedTarget = false;
        if(targetPosition == null)
        {
            if (gamepad2.dpad_left) {
                RunSlideMode(LinearSlideModes.SLIDE_UP);
            } else if (gamepad2.dpad_right) {
                RunSlideMode(LinearSlideModes.SLIDE_DOWN);
            } else {
                RunSlideMode(LinearSlideModes.SLIDE_OFF);
            }
        }
        else {
            hasReachedTarget = checkPosition(targetPosition);
            if (hasReachedTarget) {
                if(runtime.seconds() - lastNewTimer > 2) {
                    targetPosition = null;
                }
            }
        }

        WeGoUp.setTargetPosition((int)targetSlidePosition);
    }
}
