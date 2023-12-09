package org.firstinspires.ftc.teamcode.droneLauncher;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    Servo launcherServo;
    boolean launching = false;

    Telemetry telemetry;

    public DroneLauncher(HardwareMap hardwareMap, Telemetry tele)
    {
        telemetry = tele;
        launcherServo = hardwareMap.get(Servo.class, "plane");
    }

    public void Launch(boolean pressed)
    {
        if(pressed) {
            launcherServo.setPosition(1.0);
            launching = true;
        }
        else if (launching) {
            launcherServo.setPosition(0.0);
            launching = false;
        }
    }
}
