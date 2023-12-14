package org.firstinspires.ftc.teamcode.arm;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

public class ArmPositions {
    public double wristPosition;
    public double elbowPosition;
    public double sliderPosition;
    public double grabberPosition;

    public ArmPositions(double slider, double elbow, double wrist, boolean grabbing)
    {
        wristPosition = wrist;
        elbowPosition = elbow;
        sliderPosition = slider;
        if(grabbing)
        {
            grabberPosition = 1.0;
        }
        else
        {
            grabberPosition = 0.0;
        }
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
        list.add(new ArmPositions(0, 0, 0, false)); //Starting pos.
        list.add(new ArmPositions(0, 830, 0, false)); // Allows the wrist free movement into the intake
        list.add(new ArmPositions(0, 830, 0.70, false)); // Prepare to pick it up; move wrist to go directly down
    }

    public static void BuildReadyPickupToPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0,-256,.70,false)); // Lower arm/wrist into pixel holes
        list.add(new ArmPositions(0,-256,.70,true)); // Grab pixels
    }

    public static void BuildPickupToPrepareToPlace(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0,300,.70,true)); // move arm to allow for pixel adjustment
        list.add(new ArmPositions(0,300,.78,true)); // pixel adjustment to take out of intake
        list.add(new ArmPositions(0, 2060 /*2760*/ , 0.78, true)); // Raise the arm out of intake with pixels
        list.add(new ArmPositions(0, 2060 /*2760*/ , 0.23, true)); // Prepare to place the code.
    }

    public static void BuildPrepareToPlaceToReadyToPickup(List<ArmPositions> list)
    {
        list.add(new ArmPositions(0,830,.70,false)); // ReadyToPickup position; Prepare to take more pixels
    }
}
