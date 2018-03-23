package interfaces;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 23/03/2018.
 * a class of autopilot factories
 */
public class AutopilotFactory {

    public static Autopilot createAutopilot() {
        return new Autopilot() {
            @Override
            public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) {
                System.out.println("Config brake force: " + config.getRMax());
                //get the orientation used in their testbed
                System.out.println("Orientation: " + extractOrientation(inputs));
                return testOutputs;
            }

            @Override
            public AutopilotOutputs timePassed(AutopilotInputs inputs) {
                System.out.println("Orientation: " + extractOrientation(inputs));
                return testOutputs;
            }

            @Override
            public void simulationEnded() {

            }
        };
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the orientation of the drone in vector format
     */
    protected static Vector extractOrientation(AutopilotInputs inputs){
        return new Vector(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the position of the drone in vector format
     */
    protected static Vector extractPosition(AutopilotInputs inputs){
        return new Vector(inputs.getX(), inputs.getY(), inputs.getZ());
    }


    private final static AutopilotOutputs testOutputs =  new AutopilotOutputs() {
        @Override
        public float getThrust() {
            return 2000;
        }

        @Override
        public float getLeftWingInclination() {
            return 0;
        }

        @Override
        public float getRightWingInclination() {
            return 0;/*(float) (5*PI/180);*/
        }

        @Override
        public float getHorStabInclination() {
            return 0;
        }

        @Override
        public float getVerStabInclination() {
            return 0;
        }

        @Override
        public float getFrontBrakeForce() {
            return 0;
        }

        @Override
        public float getLeftBrakeForce() {
            return 0;
        }

        @Override
        public float getRightBrakeForce() {
            return 0;
        }
    };


}
