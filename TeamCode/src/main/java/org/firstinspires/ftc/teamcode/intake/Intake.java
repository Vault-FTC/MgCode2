package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor intakeMotor = null;

    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry tele)
    {
        telemetry = tele;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    public void runIntake(IntakeModes mode)
    {
        switch(mode)
        {
            case INTAKE_ON:
                intakeMotor.setPower(1.0);
                break;
            case INTAKE_OFF:
                intakeMotor.setPower(0.0);
                break;
            case INTAKE_OUT:
                intakeMotor.setPower(-1.0);
                break;
        }
    }
}
