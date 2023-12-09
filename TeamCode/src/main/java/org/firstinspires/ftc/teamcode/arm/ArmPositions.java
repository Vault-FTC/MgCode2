package org.firstinspires.ftc.teamcode.arm;

import java.util.ArrayList;
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
            grabberPosition = 0.0;
        }
        else
        {
            grabberPosition = 1.0;
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

    public static List<ArmPositions> BuildArmStartupToReadyPickup()
    {
        List<ArmPositions> list = new ArrayList<>();
        list.add(new ArmPositions(0, 0, 0, false)); //starting position
        list.add(new ArmPositions(0, 830, 0, false)); //wrist-free position
        list.add(new ArmPositions(0, 830, 0.76, false)); // ReadToPickup position
//        list.add(new ArmPositions(0, 201, 0.76, false)); // Pickup position
//        list.add(new ArmPositions(0, 400, 0.76, true)); // Pixel-Free position
//        list.add(new ArmPositions(0, 400, 0.84, true)); // PrepPixel position
//        list.add(new ArmPositions(0, 1860 /*2760*/ , 0.23, true)); // PrepToPlace position
        return list;
    }

    public static List<ArmPositions> BuildReadyPickupToPickup()
    {
        List<ArmPositions> list = new ArrayList<>();
        list.add(new ArmPositions(0,201,.67,false));
        list.add(new ArmPositions(0,201,.67,true));
        return list;
    }

    public static List<ArmPositions> BuildPickupToPrepareToPlace()
    {
        List<ArmPositions> list = new ArrayList<>();
        list.add(new ArmPositions(0,400,.76,true));
        list.add(new ArmPositions(0,400,.84,true));
        list.add(new ArmPositions(0, 1860 /*2760*/ , 0.23, true)); // PrepToPlace position
        return list;
    }

    public static List<ArmPositions> BuildPrepareToPlaceToReadyToPickup()
    {
        List<ArmPositions> list = new ArrayList<>();
        list.add(new ArmPositions(0,830,.76,false)); // ReadyToPickup position
        return list;
    }
}
