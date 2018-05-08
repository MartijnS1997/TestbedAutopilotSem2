package interfaces;

import static java.lang.Math.PI;
import Autopilot.AutoPilot;
import Helper.Vector;

/**
 * Created by Martijn on 23/03/2018.
 * a class of autopilot factories
 */
public class AutopilotFactory {

    public static Autopilot createAutopilot() {
        AutopilotCompatability.generatePath();
        return new Autopilot() {
            @Override
            public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) {
                //first run trough the compatability layer
                AutopilotConfig config_v2 = AutopilotCompatability.convertConfig(config);
                AutopilotInputs_v2 inputs_v2 = AutopilotCompatability.convertInputs(inputs, config_v2);
                Path approxPath = AutopilotCompatability.extractPath();
                autopilot_v2.setPath(approxPath);
                //generate the output
                return autopilot_v2.simulationStarted(config_v2, inputs_v2);
            }

            @Override
            public AutopilotOutputs timePassed(AutopilotInputs inputs) {
                //send the inputs trough the compatability layer
                AutopilotInputs_v2 inputs_v2 = AutopilotCompatability.convertInputs(inputs, autopilot_v2.getConfig());
                //get the outputs
                return autopilot_v2.timePassed(inputs_v2);
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

    /**
     * The autopilot we will use for our simulations
     */
    private static AutoPilot autopilot_v2 = new AutoPilot();

}
