package org.firstinspires.ftc.teamcode.arm;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

public class ArmPositions {
    public double wristPosition;
    public double elbowPosition;
    public double sliderPosition;

    public ArmPositions(double slider, double elbow, double wrist)
    {
        wristPosition = wrist;
        elbowPosition = elbow;
        sliderPosition = slider;
    }

//    public static ArmPositions[] BuildArmPositions()
//    {
//            return new ArmPositions[]
//                    {
//                      new ArmPositions(0.28, 140, 0),
//                      new ArmPositions(0.28, 800,0)
//                    };
//    }

    public static void BuildArmStartupToReadyPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0, 1200, 0));
        list.add(new ArmPositions(0, 1200, .13)); // Allows the wrist free movement into the intake
        list.add(new ArmPositions(0, 650, .13)); // Prepare to pick it up; move wrist to go directly down
        list.add(new ArmPositions(0, 650, .22)); // Prepare to pick it up; move wrist to go directly down
        list.add(new ArmPositions(0, 450, .22)); // Prepare to pick it up; move wrist to go directly down
        list.add(new ArmPositions(0, 450, .31));
        list.add(new ArmPositions(0, 130, .31));

    }

    public static void BuildReadyPickupToPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0, 140, .40));
        list.add(new ArmPositions(4000, 140, .40));
        list.add(new ArmPositions(4000, 140, .74));
        list.add(new ArmPositions(20, 140, .74));
    }

    public static void BuildPickupToPrepareToPlace(List<ArmPositions> list)
    {
        list.add(new ArmPositions(20,1300,.74)); // move arm to allow for pixel adjustment
        list.add(new ArmPositions(20,1300,.40)); // pixel adjustment to take out of intake
        list.add(new ArmPositions(20, 2750 , .40)); // Raise the arm out of intake with pixels
    }

    public static void BuildPrepareToPlaceToReadyToPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0,1200,0.25));
        list.add(new ArmPositions(0,100,0.01)); // ReadyToPickup position; Prepare to take more pixels
    }
}
