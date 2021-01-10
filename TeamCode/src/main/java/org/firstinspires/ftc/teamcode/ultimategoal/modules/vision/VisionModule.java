package org.firstinspires.ftc.teamcode.ultimategoal.modules.vision;


import org.opencv.core.Mat;

public interface VisionModule {
    /**
     * Updates the module, executing all the tasks it should complete on every iteration,
     * utilizing the module's states. This separate method allows for each module to be updated
     * on a different thread from where the states are set.
     */
    public void update(Mat inputMat);

    public boolean isOn();

    public String getName();
}
