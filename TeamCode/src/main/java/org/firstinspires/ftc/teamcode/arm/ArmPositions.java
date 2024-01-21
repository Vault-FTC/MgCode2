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
        list.add(new ArmPositions(0, 0, 0)); //Starting pos.
        list.add(new ArmPositions(200, 0, 0)); // Allows the wrist free movement into the intake
        list.add(new ArmPositions(100, 0, 0)); // Prepare to pick it up; move wrist to go directly down
    }

    public static void BuildReadyPickupToPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(300,0,0)); // Lower arm/wrist into pixel holes
        list.add(new ArmPositions(100,0,0)); // Grab pixels
    }

    public static void BuildPickupToPrepareToPlace(List<ArmPositions> list)
    {
        list.add(new ArmPositions(200,0,0)); // move arm to allow for pixel adjustment
        list.add(new ArmPositions(100,0,0)); // pixel adjustment to take out of intake
        list.add(new ArmPositions(300, 0 /*2760*/ , 0)); // Raise the arm out of intake with pixels
        list.add(new ArmPositions(0, 0 /*2760*/ , 0)); // Prepare to place the code.
    }

    public static void BuildPrepareToPlaceToReadyToPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0,0,0)); // ReadyToPickup position; Prepare to take more pixels
    }
}
