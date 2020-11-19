package org.firstinspires.ftc.teamcode.ultimategoal.util;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

/**
 * Powershot goals are numbered where the first is the closest to the centre of its colour <br/>
 * <strong>the following code will take you on an emotional roller coaster</strong><br/>
 * <i>this is your last chance to turn back and not read the file</i>
 */

public class Target {
    // Position of goals, all in inches, from the center of the robot at the front blue corner (audience, left)
    private static final double HIGH_GOAL_CENTER_HEIGHT = 33.0 + (5.0 / 2) - 0.625;
    private static final double MIDDLE_GOAL_CENTER_HEIGHT = 21.0 + (12.0 / 2) - 0.625;
    private static final double LOW_GOAL_CENTER_HEIGHT = 13.0 + (8.0 / 2) - 0.625; // Subtract to account for thickness of mat
    private static final double BLUE_GOAL_CENTER_X = 27; // Subtract to account for center of robot
    private static final double RED_GOAL_CENTER_X = 23.0 + (23.5 * 3) + (24.0 / 2) - 9;

    private static final double POWERSHOT_CENTRE_Y = 21;
    private static final double POWERSHOT_CENTRE_X = 48.75; // lol this needs to be checked, GM (i don't think) had this specific measurement, and this calculation might be off
    // these are abs of dist from centre (for each powershot)
    private static final double SHOT1_OFFSET = 3.5, SHOT2_OFFSET = 11, SHOT3_OFFSET = 18.5;

    public interface ITarget {
        /**
         * Our next target (for toggling)
         * In order: HIGH, POWERSHOT1, POWERSHOT2, POWERSHOT3
         *
         * @return The next target
         */
        ITarget next();

        /**
         * Returns an enum of the same goal selected, but for the opposite alliance colour
         *
         * @return
         */
        ITarget switchColour();

        /**
         * Returns the position of <code>this</code>, relative to (0,0) being the bottom left (blue and audience side)
         * of the field.
         *
         * @return The position of that goal, as a Point.
         * @see Point
         */
        Point getLocation();

        /**
         * Returns whether this target is a powershot or not
         *
         * @return whether this target is a powershot or not
         */
        boolean isPowershot();
    }

    public enum Red implements ITarget {
        RED_HIGH(new Point(RED_GOAL_CENTER_X, HIGH_GOAL_CENTER_HEIGHT), false),
        RED_MIDDLE(new Point(RED_GOAL_CENTER_X, MIDDLE_GOAL_CENTER_HEIGHT), false),
        RED_LOW(new Point(RED_GOAL_CENTER_X, LOW_GOAL_CENTER_HEIGHT), false),

        RED_POWERSHOT1(new Point(POWERSHOT_CENTRE_X + SHOT1_OFFSET, POWERSHOT_CENTRE_Y), true),
        RED_POWERSHOT2(new Point(POWERSHOT_CENTRE_X + SHOT2_OFFSET, POWERSHOT_CENTRE_Y), true),
        RED_POWERSHOT3(new Point(POWERSHOT_CENTRE_X + SHOT3_OFFSET, POWERSHOT_CENTRE_Y), true);

        private final Point location;
        private final boolean isPowershot;

        private Red(Point location, boolean isPowershot) {
            this.location = location;
            this.isPowershot = isPowershot;
        }

        @Override
        public Point getLocation() {
            return location;
        }

        public boolean isPowershot() {
            return isPowershot;
        }

        public ITarget next() {
            ITarget target;
            switch (name()) {
                case "RED_HIGH":
                    target = RED_POWERSHOT1;
                case "RED_POWERSHOT1":
                    target = RED_POWERSHOT2;
                case "RED_POWERSHOT2":
                    target = RED_POWERSHOT3;
                case "RED_POWERSHOT3":
                    target = RED_HIGH;
                default:
                    target = RED_HIGH;
            }
            return target;
        }

        public ITarget switchColour() {
            return Blue.values()[ordinal()];
        }
    }

    public enum Blue implements ITarget {
        BLUE_HIGH(new Point(BLUE_GOAL_CENTER_X, HIGH_GOAL_CENTER_HEIGHT), false),
        BLUE_MIDDLE(new Point(BLUE_GOAL_CENTER_X, MIDDLE_GOAL_CENTER_HEIGHT), false),
        BLUE_LOW(new Point(BLUE_GOAL_CENTER_X, LOW_GOAL_CENTER_HEIGHT), false),

        BLUE_POWERSHOT1(new Point(POWERSHOT_CENTRE_X - SHOT1_OFFSET, POWERSHOT_CENTRE_Y), true),
        BLUE_POWERSHOT2(new Point(POWERSHOT_CENTRE_X - SHOT2_OFFSET, POWERSHOT_CENTRE_Y), true),
        BLUE_POWERSHOT3((new Point(POWERSHOT_CENTRE_X - SHOT3_OFFSET, POWERSHOT_CENTRE_Y)), true);

        private final Point location;
        private final boolean isPowershot;

        Blue(Point location, boolean isPowershot) {
            this.location = location;
            this.isPowershot = isPowershot;
        }

        @Override
        public Point getLocation() {
            return location;
        }

        @Override
        public boolean isPowershot() {
            return isPowershot;
        }

        public ITarget next() {
            ITarget target;
            switch(name()) {
                case "BLUE_HIGH":
                    target = BLUE_POWERSHOT1;
                case "BLUE_POWERSHOT1":
                    target = BLUE_POWERSHOT2;
                case "BLUE_POWERSHOT2":
                    target = BLUE_POWERSHOT3;
                case "BLUE_POWERSHOT3":
                    target = BLUE_HIGH;
                default:
                    target = BLUE_HIGH;
            }
            return target;
        }

        public ITarget switchColour() {
            return Red.values()[ordinal()];
        }
    }
}



