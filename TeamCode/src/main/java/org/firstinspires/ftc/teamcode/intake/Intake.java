package org.firstinspires.ftc.teamcode.intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoOpModes.SimpleOpenCVColor;

public class Intake {
    private DcMotorEx intakeMotor = null;
    public DcMotor intakeSpinner = null;

    private final int INTAKE_COUNTS_PER_ROTATION = 288;

    int currentTargetPosition = 0;



    IntakeModes currentMode = IntakeModes.INTAKE_OFF;
    IntakeModes lastMode = IntakeModes.INTAKE_OFF;

    Telemetry telemetry;


    public Intake(HardwareMap hardwareMap, Telemetry tele)
    {
        telemetry = tele;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeSpinner = hardwareMap.get(DcMotor.class, "intake_spinner");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 0.1,0));
        intakeMotor.setPower(1);
    }

    public void changeIntake(IntakeModes mode)
    {
        currentMode = mode;
        telemetry.addData("Intake: ", Math.abs(currentTargetPosition - intakeMotor.getCurrentPosition()));
        if(Math.abs(currentTargetPosition - intakeMotor.getCurrentPosition()) < 40) {
            switch (currentMode) {
                case INTAKE_ON:
                    currentTargetPosition = currentTargetPosition - (INTAKE_COUNTS_PER_ROTATION);
                    intakeMotor.setTargetPosition(currentTargetPosition);
                    break;
                case INTAKE_OFF:
                    break;
                case INTAKE_OUT:
                    currentTargetPosition = currentTargetPosition + (INTAKE_COUNTS_PER_ROTATION);
                    intakeMotor.setTargetPosition(currentTargetPosition);
                    break;
            }
        }
    }

    public void runIntake()
    {
        if(lastMode != currentMode) {
            switch (currentMode) {
                case INTAKE_ON:
                    intakeMotor.setPower(-1.0);
                    break;
                case INTAKE_OUT:
                    intakeSpinner.setPower(1.0);
                    break;
                case INTAKE_OFF:
                    intakeSpinner.setPower(0);
                    break;
            }
            lastMode = currentMode;
        }
//        switch(currentMode)
//        {
//            case INTAKE_ON:
//                intakeMotor.setPower(0.8);
//                intakeSpinner.setPower(1.0);
//                break;
//            case INTAKE_OFF:
//                int remainder = Math.abs(intakeMotor.getCurrentPosition() % INTAKE_COUNTS_PER_ROTATION);
//                telemetry.addData("Intake Mod: ", remainder);
//                if(remainder < 250 && remainder > 240)
//                {
//                    intakeMotor.setPower(0.0);
//                    intakeSpinner.setPower(0.0);
//                }
//                telemetry.addData("Intake :", intakeMotor.getCurrentPosition());
//                break;
//            case INTAKE_OUT:
//                intakeMotor.setPower(-1.0);
//                intakeSpinner.setPower(-1.0);
//                break;
//        }
    }
}
