package interfaces;

import Helper.WorldGenerator;

import java.io.*;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

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
    public static AutopilotInputs_v2 convertInputs(AutopilotInputs inputs, AutopilotConfig config){
        float xPos = inputs.getX();
        float yPos = inputs.getY();
        float zPos = inputs.getZ();

        float heading = inputs.getHeading();
        float pitch = inputs.getPitch();
        float roll = inputs.getRoll();

        byte[] image = deepCopyConvertImage(inputs, config);

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
     * @param config the configuration of the autopilot
     * @return a deep copy of the given image
     */
    private static byte[] deepCopyConvertImage(AutopilotInputs inputs, AutopilotConfig config) {
        byte[] tempImage = inputs.getImage();
        byte[] copyImage = new byte[tempImage.length];

        for(int i = 0; i != copyImage.length; i++){
            copyImage[i] = tempImage[i];
        }

        return flipImageUpDown(copyImage, config.getNbRows(), config.getNbColumns());
    }

    /**
     * Flips the given image upside down (mirrored across the center axis)
     * @param image the image to mirror
     * @param rows the nb of rows in the image
     * @param columns the nb of columns in the image
     */
    private static byte[] flipImageUpDown(byte[] image, int rows, int columns){
        int bytesPerPixel = 3;
        //byte buffer
        byte[] flippedImage = new byte[image.length];
        //buffers for two pixels

        for(int i = 0; i <= rows/2; i++){
            for(int j = 0; j != columns; j++){
                //select the two elements to be swapped
                for(int k = 0; k != bytesPerPixel; k++){
                    int indexPixel1 = convertIndexToInt(i, j, k, columns, bytesPerPixel);
                    int indexPixel2 = convertIndexToInt(rows - 1 - i, j, k, columns, bytesPerPixel);
                    byte pixelByte1 = image[indexPixel1];
                    byte pixelByte2 = image[indexPixel2];
                    //swap them in the original image
                    flippedImage[indexPixel1] = pixelByte2;
                    flippedImage[indexPixel2] = pixelByte1;
                }
            }
        }

        return flippedImage;
    }

    /**
     * Converts a given three dimensional index to a one dimensional index for array access
     * @param i the row to access
     * @param j the column to access
     * @param k the rgb value to access
     * @param columns the number of columns the 3d array has
     * @param bytesPerPixel the number of bytes per pixel
     * @return a 1d index: i * columns*bpp + j*bpp + k
     */
    private static int convertIndexToInt(int i, int j, int k, int columns, int bytesPerPixel){
        return  i*columns*bytesPerPixel+ j*bytesPerPixel + k;
    }

    /**
     * Generates a random path written to the current working directory, to be read from by the autopilot
     */
    public static void generatePath(){
        int nbCubes = 5;
        WorldGenerator generator = new WorldGenerator(nbCubes);
        generator.createPath(BASE_VECTOR, REAL_PATH, APPROX_PATH);
    }

    /**
     * Extracts the path from the runtime environment at name: "randomPath"
     * @return the path located in the file randomPath in the execution environment
     */
    public static Path extractPath(){
        java.nio.file.Path apPathDirectory = Paths.get(System.getProperty("user.dir"), APPROX_PATH);
        System.out.println("Current Directory: " + apPathDirectory);
        File pathFile = new File(apPathDirectory.toUri());
        List<Vector> pathEntries = new ArrayList<>();
        //open a stream to read
        try {
            BufferedReader fileReader = new BufferedReader(new FileReader(pathFile));
            String line;

            while((line = fileReader.readLine())!= null){
                String[] lineElems = line.split(" ");
                float xPos = Float.parseFloat(lineElems[0]);
                float yPos = Float.parseFloat(lineElems[1]);
                float zPos = Float.parseFloat(lineElems[2]);
                Vector pathElem = new Vector(xPos, yPos, zPos);
                System.out.println("extracted element: " + pathElem);
                pathEntries.add(pathElem);
            }
            fileReader.close();
            System.out.println("Exited retrieval loop");
            return convertToPath(pathEntries);
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("The size of the cube list: " + pathEntries.size());
        return convertToPath(pathEntries);
    }

    /**
     * Converts a given list of vectors into a path object as specified in the autopilot interfaces
     * @param vectors the vectors to create a path from
     * @return a path with the same positions as the provided vectors
     */
    private static Path convertToPath(List<Vector> vectors){
        int pathLen = vectors.size();
        float[] xPos = new float[pathLen];
        float[] yPos = new float[pathLen];
        float[] zPos = new float[pathLen];

        for(int index = 0; index != pathLen; index++){
            Vector vector = vectors.get(index);
            xPos[index] = vector.getxValue();
            yPos[index] = vector.getyValue();
            zPos[index] = vector.getzValue();
        }

        //return the converted path
        return new Path(){
            @Override
            public float[] getX() {
                return xPos;
            }

            @Override
            public float[] getY() {
                return yPos;
            }

            @Override
            public float[] getZ() {
                return zPos;
            }
        };
    }

    private final static String REAL_PATH = "realPath";
    private final static String APPROX_PATH = "randomizedPath";
    private final static Vector BASE_VECTOR = new Vector(0, 30f, -700f);
}
