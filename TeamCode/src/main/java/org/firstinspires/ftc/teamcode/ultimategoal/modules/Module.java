package org.firstinspires.ftc.teamcode.ultimategoal.modules;


public interface Module {
    /**
     * Initializes the module. This includes setting up all motors/servos
     */
    void initModules();

    /**
     * Any logic for initialization that requires repetitive calling should go here.
     *
     * @return whether or not the module has completed initializing, so that this method can stop
     *         being called
     */
    default boolean initAsync() {
        return true;
    }

    default void onStart() {
    }

    default void onClose() {
    }

    /**
     * Updates the module, executing all the tasks it should complete on every iteration, utilizing
     * the module's states. This separate method allows for each module to be updated on a different
     * thread from where the states are set.
     */
    void update();

    boolean isOn();

    String getName();
}
