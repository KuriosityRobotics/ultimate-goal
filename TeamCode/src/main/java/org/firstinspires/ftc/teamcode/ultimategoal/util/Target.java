package org.firstinspires.ftc.teamcode.ultimategoal.util;

import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

/**
 * Powershot goals are numbered where the first is the closest to the centre of its colour <br/>
 * <strong>the following code will take you on an emotional roller coaster</strong><br/>
 * <i>this is your last chance to turn back and not read the file</i>
 */

public class Target {
    // Position of goals, all in inches, from the center of the robot at the front blue corner (audience, left)
    private static final double BLUE_GOAL_CENTER_X = 27; // Subtract to account for center of robot
    private static final double RED_GOAL_CENTER_X = 23.0 + (23.5 * 3) + (24.0 / 2) - 9;
    //    private static final double GOAL_CENTER_Y = 6 * 24.0 - (0.5 * 2) - 9;
    private static final double GOAL_CENTER_Y = (24 * 6) - 9;

    private static final double POWERSHOT_CENTRE_X = (24 - 0.5) * 3 - 9;
    // these are abs of dist from centre (for each powershot)
    private static final double SHOT1_OFFSET = 8, SHOT2_OFFSET = 16, SHOT3_OFFSET = 20; // todo force tuned ooga booga
//    private static final double SHOT1_OFFSET = 3.5, SHOT2_OFFSET = 11, SHOT3_OFFSET = 18.5;

    public interface ITarget {
        /**
         * Our next target (for toggling) In order: HIGH, POWERSHOT1, POWERSHOT2, POWERSHOT3
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

        ITarget getAllianceHigh();

        /**
         * Returns the position of <code>this</code>, relative to (0,0) being the bottom left (blue
         * and audience side) of the field.
         *
         * @return The position of that goal, as a Point.
         * @see Point
         */
        Point getLocation();

        String name();

        /**
         * Returns whether this target is a powershot or not
         *
         * @return whether this target is a powershot or not
         */
        boolean isPowershot();
    }

    public enum Red implements ITarget {
        RED_HIGH(new Point(RED_GOAL_CENTER_X, GOAL_CENTER_Y), false),
        //        RED_MIDDLE(new Point(RED_GOAL_CENTER_X, GOAL_CENTER_Y), false),
        //        RED_LOW(new Point(RED_GOAL_CENTER_X, GOAL_CENTER_Y), false),

        RED_POWERSHOT1(new Point(POWERSHOT_CENTRE_X + SHOT1_OFFSET, GOAL_CENTER_Y), true),
        RED_POWERSHOT2(new Point(POWERSHOT_CENTRE_X + SHOT2_OFFSET, GOAL_CENTER_Y), true),
        RED_POWERSHOT3(new Point(POWERSHOT_CENTRE_X + SHOT3_OFFSET, GOAL_CENTER_Y), true);

        private final Point location;
        private final boolean isPowershot;

        Red(Point location, boolean isPowershot) {
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
                    break;
                case "RED_POWERSHOT1":
                    target = RED_POWERSHOT2;
                    break;
                case "RED_POWERSHOT2":
                    target = RED_POWERSHOT3;
                    break;
                case "RED_POWERSHOT3":
                    target = RED_HIGH;
                    break;
                default:
                    target = RED_HIGH;
                    break;
            }
            return target;
        }

        public ITarget getAllianceHigh() {
            return RED_HIGH;
        }

        public ITarget switchColour() {
            return Blue.values()[ordinal()];
        }
    }

    public enum Blue implements ITarget {
        BLUE_HIGH(new Point(BLUE_GOAL_CENTER_X, GOAL_CENTER_Y), false),
        //        BLUE_MIDDLE(new Point(BLUE_GOAL_CENTER_X, GOAL_CENTER_Y), false),
        //        BLUE_LOW(new Point(BLUE_GOAL_CENTER_X, GOAL_CENTER_Y), false),

        BLUE_POWERSHOT1(new Point(POWERSHOT_CENTRE_X - SHOT1_OFFSET, GOAL_CENTER_Y), true),
        BLUE_POWERSHOT2(new Point(POWERSHOT_CENTRE_X - SHOT2_OFFSET, GOAL_CENTER_Y), true),
        BLUE_POWERSHOT3((new Point(POWERSHOT_CENTRE_X - SHOT3_OFFSET, GOAL_CENTER_Y)), true);

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
            switch (name()) {
                case "BLUE_HIGH":
                    target = BLUE_POWERSHOT1;
                    break;
                case "BLUE_POWERSHOT1":
                    target = BLUE_POWERSHOT2;
                    break;
                case "BLUE_POWERSHOT2":
                    target = BLUE_POWERSHOT3;
                    break;
                case "BLUE_POWERSHOT3":
                    target = BLUE_HIGH;
                    break;
                default:
                    target = BLUE_HIGH;
                    break;
            }
            return target;
        }

        public ITarget getAllianceHigh() {
            return BLUE_HIGH;
        }

        public ITarget switchColour() {
            return Red.values()[ordinal()];
        }
    }
}



