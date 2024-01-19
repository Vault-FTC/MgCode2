package org.firstinspires.ftc.teamcode.LinearSlidw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LinearSlide {
    private DcMotor WeGoUp = null;
    LinearSlideModes CurrentSlideMode = LinearSlideModes.SLIDE_OFF;
    LinearSlideModes LastSlideMode = LinearSlideModes.SLIDE_OFF;
    public LinearSlide(HardwareMap hardwareMap)
    {
        WeGoUp = hardwareMap.get(DcMotor.class, "WeGoUp_motor");

    }
    public void RunSlideMode(LinearSlideModes mode)
    {
        CurrentSlideMode = mode;
        if (CurrentSlideMode != LastSlideMode) {
            LastSlideMode = CurrentSlideMode;
            switch (CurrentSlideMode) {
                case SLIDE_UP:
                    WeGoUp.setPower(0.5);
                    break;
                case SLIDE_OFF:
                    WeGoUp.setPower(0);
                    break;
                case SLIDE_DOWN:
                    WeGoUp.setPower(-0.5);
                    break;
            }
        }
    }
}
