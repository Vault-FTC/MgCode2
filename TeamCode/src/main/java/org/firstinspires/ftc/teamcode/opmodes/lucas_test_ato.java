package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SimpleDrive;

public class lucas_test_ato {
    @Autonomous(name = "Lucas Test Auto", group = "Iterative Opmode")
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
}

// 😴z z Z Z  Z   Z