package frc.robot.commands;

public class AutoConfig {
    //relative to the alliance wall
    public enum ShootingRegion{
        LEFT,
        CENTER,
        RIGHT
    }
    public enum PickupRegion{
        DEPOT,
        LEFT,
        RIGHT
    }
    // change all constants as needed

    //Shoots starting 8 then goes to pickup from:
    public static PickupRegion pickupLocationOne = PickupRegion.RIGHT;
    // through all the balls or halfway
    public static boolean followThroughOne = false;
    //then come back to:
    public static ShootingRegion shootingPositionOne = ShootingRegion.CENTER;
    // do we climb
    public static boolean climb = false;

}
