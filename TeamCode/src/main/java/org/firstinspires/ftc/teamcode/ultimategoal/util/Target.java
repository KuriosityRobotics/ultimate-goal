package org.firstinspires.ftc.teamcode.ultimategoal.util;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

/**
 * Powershot goals are numbered where the first is the closest to the centre of its colour <br/>
 * <strong>the following code will take you on an emotional rollercoaster</strong><br/>
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
         * In order: HIGH, MIDDLE, LOW, POWERSHOT1, POWERSHOT2, POWERSHOT3
         * @return The next target
         */
        ITarget next();

        /**
         * Returns an enum of the same goal selected, but for the opposite alliance colour
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

        String name();
    }

    public enum Red implements ITarget {
        RED_HIGH(new Point(RED_GOAL_CENTER_X, HIGH_GOAL_CENTER_HEIGHT)),
        RED_MIDDLE(new Point(RED_GOAL_CENTER_X, MIDDLE_GOAL_CENTER_HEIGHT)),
        RED_LOW(new Point(RED_GOAL_CENTER_X, LOW_GOAL_CENTER_HEIGHT)),

        RED_POWERSHOT1(new Point(POWERSHOT_CENTRE_X + SHOT1_OFFSET, POWERSHOT_CENTRE_Y)),
        RED_POWERSHOT2(new Point(POWERSHOT_CENTRE_X + SHOT2_OFFSET, POWERSHOT_CENTRE_Y)),
        RED_POWERSHOT3(new Point(POWERSHOT_CENTRE_X + SHOT3_OFFSET, POWERSHOT_CENTRE_Y));

        private final Point location;
        Red(Point location) {
            this.location = location;
        }
        @Override
        public Point getLocation() {
            return location;
        }

        Red[] vals = values();
        public ITarget next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public ITarget switchColour() {
            return Blue.values()[ordinal()];
        }
    }

    public enum Blue implements ITarget {
        BLUE_HIGH(new Point(BLUE_GOAL_CENTER_X, HIGH_GOAL_CENTER_HEIGHT)),
        BLUE_MIDDLE(new Point(BLUE_GOAL_CENTER_X, MIDDLE_GOAL_CENTER_HEIGHT)),
        BLUE_LOW(new Point(BLUE_GOAL_CENTER_X, LOW_GOAL_CENTER_HEIGHT)),
        BLUE_POWERSHOT1(new Point(POWERSHOT_CENTRE_X - SHOT1_OFFSET, POWERSHOT_CENTRE_Y)),
        BLUE_POWERSHOT2(new Point(POWERSHOT_CENTRE_X - SHOT2_OFFSET, POWERSHOT_CENTRE_Y)),
        BLUE_POWERSHOT3((new Point(POWERSHOT_CENTRE_X - SHOT3_OFFSET, POWERSHOT_CENTRE_Y)));

        private final Point location;
        Blue(Point location) {
            this.location = location;
        }
        @Override
        public Point getLocation() {
            return location;
        }

        Blue[] vals = values();

        public ITarget next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public ITarget switchColour() {
            return Red.values()[ordinal()];
        }
    }
}



