package frc.robot.commands;

public class AutoConfig {
    //relative to the alliance wall
    public enum ShootingPosition{
        LEFT,
        CENTER,
        RIGHT
    }
    public enum PickupLocation{
        DEPOT,
        LEFT,
        RIGHT
    }
    // change all constants as needed

    //Shoots starting 8 then goes to pickup from:
    public static PickupLocation pickupLocationOne = PickupLocation.LEFT;
    // through all the balls or halfway
    public static boolean followThroughOne = true;
    //then come back to:
    public static ShootingPosition shootingPositionOne = ShootingPosition.RIGHT;
    // do we climb
    public static boolean climb = false;

}
