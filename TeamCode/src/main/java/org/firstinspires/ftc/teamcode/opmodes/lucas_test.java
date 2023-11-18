package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SimpleDrive;

@TeleOp(name="Lucas Test", group="Iterative Opmode")
@Disabled
public class lucas_test extends OpMode {

    SimpleDrive drive;

    @Override
    public void init() {
        drive = new SimpleDrive();
    }

    @Override
    public void loop() {
        drive.loop();
    }
}
