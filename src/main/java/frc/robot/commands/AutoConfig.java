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
    public PickupLocation pickupLocationOne = PickupLocation.DEPOT;
    //then come back to:
    public ShootingPosition shootingPositionTwo = ShootingPosition.LEFT;
    // do we climb
    public boolean climb = false;

}
