package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class RingManager implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    public int ringsInIntake; // will implement later
    public int ringsInHopper;
    public int ringsInShooter;

    public boolean willAutoRaise;
    public boolean willAutoShoot;

    public int ringsToAutoRaise = 1;

    // for incoming ring detection
    private double timeLastPass;
    private boolean seenRing = false;
    private int distanceSensorPasses = 0;

    private boolean readyToPush;
    private long delayStartTime;

    private static final int INTAKE_STOP_DELAY_TIME = 600;
    private static final int PUSH_DELAY_TIME = 800;


    public RingManager(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);

        ringsInIntake = 0;
        ringsInHopper = 0; // will change to 3 later b/c autonomous starts w 3 rings
        ringsInShooter = 0;

        willAutoRaise = true;
        willAutoShoot = true;

        readyToPush = false;

        this.timeLastPass = robot.getCurrentTimeMilli();
    }

    public void initModules() {}

    public void update() {

        if (ringsInShooter < 0){
            ringsInShooter = 0;
        }

        detectIncomingRingPasses();
        detectHopperPush();
        detectShooterIndex();
    }


    private void detectIncomingRingPasses(){
        long currentTime = robot.getCurrentTimeMilli();

        double voltage = robot.intakeModule.intakeDistanceVoltage;

        if (Math.abs(2.2 - voltage) < Math.abs(1.1 - voltage)) {
            if (!seenRing)
                seenRing = true; // we need to track when the sensor starts seeing it, and when it finishes

        } else if (seenRing) {
            if (robot.intakeModule.intakeBottom.getPower() > 0) // outtaking or intaking ?
                distanceSensorPasses = distanceSensorPasses + 1;
            else
                distanceSensorPasses = distanceSensorPasses - 1;
            seenRing = false;

        }
        if (currentTime - timeLastPass > 200 && (distanceSensorPasses & 1) == 1 && seenRing) { // emergency in case it somehow gets out of sync:  timeout after two seconds & round up
            distanceSensorPasses++;
        }
        timeLastPass = currentTime;

        ringsInHopper = (int)(distanceSensorPasses/2);

        if (ringsInHopper == ringsToAutoRaise){
            if (!readyToPush){
                readyToPush = true;
                delayStartTime = currentTime;
            }
        }

        //sus
        if (willAutoRaise){
            if (currentTime > delayStartTime + PUSH_DELAY_TIME){
                if (ringsInHopper == ringsToAutoRaise){
                    robot.shooter.deliverRings();
                    ringsInShooter = ringsInHopper;
                    distanceSensorPasses = 0;
                    readyToPush = false;
                }
            }
        }
    }

    private HopperModule.HopperPosition oldHopperPosition = HopperModule.HopperPosition.LOWERED;
    public void detectHopperPush(){

        HopperModule.HopperPosition currentHopperPosition = robot.shooter.hopperModule.currentHopperPosition;

        // when hopper push is finished move rings
        if (oldHopperPosition == HopperModule.HopperPosition.TRANSITIONING && currentHopperPosition == HopperModule.HopperPosition.LOWERED){

            // sus, except this one is even more sus, dirty fix srry programming team :)
            if (!willAutoRaise){
                ringsInShooter = ringsInHopper;
                distanceSensorPasses = 0;
            }

            //sus
            if(willAutoShoot){
                robot.shooter.queueIndex(robot.ringManager.ringsInShooter);
            }
        }

        oldHopperPosition = currentHopperPosition;
    }

    private ShooterModule.IndexerPosition oldIndexerPosition = ShooterModule.IndexerPosition.RETRACTED;
    public void detectShooterIndex() {
        ShooterModule.IndexerPosition currentIndexerPosition = robot.shooter.shooterModule.indexerPosition;

        // when indexer has returned from an index the shooter loses a ring
        if (oldIndexerPosition ==  ShooterModule.IndexerPosition.PUSHED && currentIndexerPosition == ShooterModule.IndexerPosition.RETRACTED){
            ringsInShooter --;
        }

        oldIndexerPosition = currentIndexerPosition;
    }

    public boolean canIntake(){

        long currentTime = robot.getCurrentTimeMilli();
        if (ringsInHopper + ringsInShooter < 3){
            return true;
        } else if (currentTime < delayStartTime + INTAKE_STOP_DELAY_TIME){
            return true;
        }
        return false;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Ring passes: " + distanceSensorPasses);
        data.add("Rings in intake: " + ringsInIntake);
        data.add("Rings in hopper: " + ringsInHopper);
        data.add("Rings in shooter: " + ringsInShooter);
        data.add("Will Auto Raise:  " + willAutoRaise);
        data.add("Will Auto Shoot: " + willAutoShoot);
        return data;
    }

    public void setDistanceSensorPasses(int distanceSensorPasses) {
        this.distanceSensorPasses = distanceSensorPasses;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "RingManager";
    }
}
