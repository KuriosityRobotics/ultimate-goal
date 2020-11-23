package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class ModuleCollection implements Module {
    protected Module[] modules;

    @Override
    public void initModules() {
        for (Module module : modules) {
            module.initModules();
        }
    }

    ArrayList<Module> toInitModules = new ArrayList<>();

    @Override
    public boolean initCycle() {
        if (toInitModules.isEmpty()) {
            toInitModules = new ArrayList<>(Arrays.asList(modules));
        }

        for (Module module : toInitModules) {
            if (module.initCycle()) {
                toInitModules.remove(module);
            }
        }

        return toInitModules.isEmpty();
    }

    @Override
    public void update() {
        for (Module module : modules) {
            if (module.isOn()) {
                module.update();
            }
        }
    }
}
