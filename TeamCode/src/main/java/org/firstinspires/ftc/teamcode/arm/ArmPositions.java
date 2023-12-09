package org.firstinspires.ftc.teamcode.arm;

public class ArmPositions {
    public double wristPosition;
    public double elbowPosition;
    public double sliderPosition;

    public ArmPositions(double wrist, double elbow, double slider)
    {
        wristPosition = wrist;
        elbowPosition = elbow;
        sliderPosition = slider;
    }

    public static ArmPositions[] BuildArmPositions()
    {
            return new ArmPositions[]
                    {
                      new ArmPositions(0.28, 140, 0),
                      new ArmPositions(0.28, 800,0)
                    };
    }
}
