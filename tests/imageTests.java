import interfaces.AutopilotCompatability;
import org.junit.Test;

import java.util.Arrays;

/**
 * Created by Martijn on 24/03/2018.
 */
public class imageTests {

    @Test
    public void testImageFlip(){
        int rows = 4;
        int columns = 4;
        int bpp = 3;
        byte[] image1 = new byte[rows*columns*bpp];
        for(int i = 0; i !=image1.length; i++){
            image1[i] = (byte) i;
        }
        //System.out.println(Arrays.toString(image1));

        for(int i = 0; i != rows; i++){
            for(int j = 0; j != columns; j++){
                for(int k = 0; k!= bpp; k++){
                    System.out.print(image1[convertIndexToInt(i, j, k, columns, bpp)]);
                    System.out.print(" ");
                }
            }
            System.out.println();
        }
        System.out.println();
        byte[] res = flipImageUpDown(image1, rows,columns);
        for(int i = 0; i != rows; i++){
            for(int j = 0; j != columns; j++){
                for(int k = 0; k!= bpp; k++){
                    System.out.print(res[convertIndexToInt(i, j, k, columns, bpp)]);
                    System.out.print(" ");
                }
            }
            System.out.println();
        }
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
     * @return a 1d index: i * columns + j*bpp + k
     */
    private static int convertIndexToInt(int i, int j, int k, int columns, int bytesPerPixel){
        return i*columns*bytesPerPixel+ j*bytesPerPixel + k;
    }

    @Test
    public void testPathExtract(){
        AutopilotCompatability.extractPath();
    }

}
