package interfaces;

/**
 * Created by Martijn on 23/03/2018.
 * a class of static methods to make the autopilot inputs compatible
 */
public class AutopilotCompatability {

    /**
     * Converts the inputs of the autopilot testbed to an input for our autopilot
     * @param inputs the inputs to convert
     * @return a deep copy of the inputs provided converted to the AutopilotInputs v2
     */
    public static AutopilotInputs_v2 convertInputs(AutopilotInputs inputs){
        float xPos = inputs.getX();
        float yPos = inputs.getY();
        float zPos = inputs.getZ();

        float heading = inputs.getHeading();
        float pitch = inputs.getPitch();
        float roll = inputs.getRoll();

        byte[] image =deepCopyImage(inputs);

        float elapsedTime = inputs.getElapsedTime();

        return new AutopilotInputs_v2() {
            @Override
            public byte[] getImage() {
                return image;
            }

            @Override
            public float getX() {
                return xPos;
            }

            @Override
            public float getY() {
                return yPos;
            }

            @Override
            public float getZ() {
                return zPos;
            }

            @Override
            public float getHeading() {
                return heading;
            }

            @Override
            public float getPitch() {
                return pitch;
            }

            @Override
            public float getRoll() {
                return roll;
            }

            @Override
            public float getElapsedTime() {
                return elapsedTime;
            }
        };
    }

    /**
     * takes a deep copy of the given configuration of the drone
     * @param config the configuration used by the testbed
     * @return a deep copy of the config of the drone
     */
    public static AutopilotConfig convertConfig(AutopilotConfig config){
        String droneID = config.getDroneID();
        float gravity= config.getGravity();
        float wingX= config.getWingX();
        float tailSize= config.getTailSize();
        float wheelY= config.getWheelY();
        float frontWheelZ= config.getFrontWheelZ();
        float rearWheelZ= config.getRearWheelZ();
        float rearWheelX= config.getRearWheelX();
        float tyreSlope= config.getTyreSlope();
        float dampSlope= config.getDampSlope();
        float tyreRadius= config.getTyreRadius();
        float rMax= config.getRMax();
        float fcMax= config.getFcMax();
        float engineMass= config.getEngineMass();
        float wingMass= config.getWingMass();
        float tailMass= config.getTailMass();
        float maxThrust= config.getMaxThrust();
        float maxAOA= config.getMaxAOA();
        float wingLiftSlope= config.getWingLiftSlope();
        float horStabLiftSlope= config.getHorStabLiftSlope();
        float verStabLiftSlope= config.getVerStabLiftSlope();
        float horizontalAngleOfView= config.getHorizontalAngleOfView();
        float verticalAngleOfView= config.getVerticalAngleOfView();
        int nbColumns= config.getNbColumns();
        int nbRows= config.getNbRows();
        return new AutopilotConfig() {
            @Override
            public String getDroneID() {
                return droneID;
            }

            @Override
            public float getGravity() {
                return gravity;
            }

            @Override
            public float getWingX() {
                return wingX;
            }

            @Override
            public float getTailSize() {
                return tailSize;
            }

            @Override
            public float getWheelY() {
                return wheelY;
            }

            @Override
            public float getFrontWheelZ() {
                return frontWheelZ;
            }

            @Override
            public float getRearWheelZ() {
                return rearWheelZ;
            }

            @Override
            public float getRearWheelX() {
                return rearWheelX;
            }

            @Override
            public float getTyreSlope() {
                return tyreSlope;
            }

            @Override
            public float getDampSlope() {
                return dampSlope;
            }

            @Override
            public float getTyreRadius() {
                return tyreRadius;
            }

            @Override
            public float getRMax() {
                return rMax;
            }

            @Override
            public float getFcMax() {
                return fcMax;
            }

            @Override
            public float getEngineMass() {
                return engineMass;
            }

            @Override
            public float getWingMass() {
                return wingMass;
            }

            @Override
            public float getTailMass() {
                return tailMass;
            }

            @Override
            public float getMaxThrust() {
                return maxThrust;
            }

            @Override
            public float getMaxAOA() {
                return maxAOA;
            }

            @Override
            public float getWingLiftSlope() {
                return wingLiftSlope;
            }

            @Override
            public float getHorStabLiftSlope() {
                return horStabLiftSlope;
            }

            @Override
            public float getVerStabLiftSlope() {
                return verStabLiftSlope;
            }

            @Override
            public float getHorizontalAngleOfView() {
                return horizontalAngleOfView;
            }

            @Override
            public float getVerticalAngleOfView() {
                return verticalAngleOfView;
            }

            @Override
            public int getNbColumns() {
                return nbColumns;
            }

            @Override
            public int getNbRows() {
                return nbRows;
            }
        };
    }

    /**
     * Takes a deep copy of the image contents
     * @param inputs the inputs of the autopilot
     * @return a deep copy of the given image
     */
    private static byte[] deepCopyImage(AutopilotInputs inputs) {
        byte[] tempImage = inputs.getImage();
        byte[] copyImage = new byte[tempImage.length];

        for(int i = 0; i != copyImage.length; i++){
            copyImage[i] = tempImage[i];
        }

        return copyImage;
    }
}
