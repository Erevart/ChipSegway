package segway;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.io.IOException;
import java.io.DataInputStream;
import java.io.Serializable;

import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.Image;
import lejos.internal.ev3.EV3GraphicsLCD;


/**
 * Provides support for in memory images.
 * The format of the bitmap is in standard leJOS format (so aligned for use on
 * EV3 LCD display). There is one bit per pixel. The pixels are packed into bytes
 * with each byte spanning 8 scan lines. The least significant bit of each byte
 * is the pixel for the top most scan line, the most significant bit is the
 * 8th scan line. Values of 1 represent black. 0 white. This class implements a
 * sub set of the standard lcdui Image class. Only mutable images are supported
 * and the ARGB methods are not available.
 * 
 * TODO: This file needs to be updated to match the EV3 image format.
 * @author Andre Nijholt & Andy Shaw
 */
public class LegoImage
{
		
    private final int width;
    private final int height;
    private final byte[] data;

    /**
     * Create an image using an already existing byte array. The byte array is
     * used to store the image data. The array may already be initialized with
     * image data.
     * <br>Note: This is a non standard constructor.
     * @param width width of the image
     * @param height height of the image
     * @param data The byte array to be used for image store/
     */
    public LegoImage(byte width, byte height, byte[] data)
    {
        this.width = width;
        this.height = height;
        this.data = data;
    }

    /**
     * Create ablank image of the requested size.
     * @param width
     * @param height
     * @return Returns the new image.
     */
    public static Image createLegoImage(int width, int height)
    {
        byte[] imageData = new byte[width * (height + 7) / 8];
        return new Image(width, height, imageData);
    }

    /**
     * Read image from file. An image file has the following format:
     * <table border="1">
     * <tr>
     * <th>1st byte - 4th byte</th>
     * <th>5th byte - 8th byte</th>
     * <th>9th byte</th>
     * <th>10th byte ....</th>
     * </tr>
     * <tr>
     * <td><i>image-width (int)</i></td>
     * <td><i>image-height (int)</i></td>
     * <td><code>0x00</code>(<i>image data delimit</i>)</td>
     * <td><i>byte image data</i>....</td>
     * </table>
     * <p>
     * For example:
     * </p>
     * After a file with content
     * <table border="1">
     * <tr>
     * <th>width (int)</th>
     * <th>height (int)</th>
     * <th>delimit</th>
     * <th colspan="3">byte data</th>
     * </tr>
     * <tr>
     * <td>00 00 00 03</td>
     * <td>00 00 00 05</td>
     * <td>00</td>
     * <td>00</td>
     * <td>02</td>
     * <td>1f</td>
     * </tr>
     * </table>
     * was read, this method will return an object which is equivalent to
     * <div style="margin-left:4em;"><code>new Image(3, 5, new byte[] {(byte)0x00, (byte)0x02, (byte)0x1f})</code></div>
     * @param s The input stream for the image file.
     * @return an ev3 image object.
     * @throws IOException if an input or output error occurs or file format is not correct.
     * @see Image
     * @see Image#Image(int, int, byte[])
     */
    public static void displayLegoImage(String s, int transformation)
    {
    	FileInputStream inFile = null;
    	DataInputStream in = null;
    	int size;
        int  w, h;
    	
		try {
			inFile = new FileInputStream(new File(s));
	        in = new DataInputStream(inFile);
	        
	        w = in.readUnsignedByte();
	        h = in.readUnsignedByte();
	        size = (h * w) / 8;
	        
	        // Valor máximo de las imágenes de lego 2944 bytes.
	        byte[] imageData = new byte[2944];
	        in.readFully(imageData, 0, size);
	        
	        GraphicsLCD g = BrickFinder.getDefault().getGraphicsLCD();
	        g.drawRegion(new Image(w,h, imageData), 0, 0, w-1, h-1, transformation, 0, 0, 0);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		finally{
			try {
				if (inFile != null)
					inFile.close();
				if (in != null)
					in.close();
			} catch(IOException e){}
		}
        
        
        return;
    }

    /**
     * Creates a new image based upon the transformed region of another image
     * @param image Source image
     * @param x x co-ordinate of the source region
     * @param y y co-ordinate of the source region
     * @param w width of the source region
     * @param h height of the source region
     * @param transform Transform to be applied
     * @return New image
     */
    public static Image displayLegoImage(Image image, int x, int y, int w, int h, int transform)
    {
    	int ow = w;
    	int oh = h;
        // Work out what shape the new image will be...
        switch (transform)
        {
            case GraphicsLCD.TRANS_MIRROR:
            case EV3GraphicsLCD.TRANS_MIRROR_ROT180:
            case EV3GraphicsLCD.TRANS_ROT180:
            case EV3GraphicsLCD.TRANS_NONE:
                break;
            case EV3GraphicsLCD.TRANS_MIRROR_ROT270:
            case EV3GraphicsLCD.TRANS_MIRROR_ROT90:
            case EV3GraphicsLCD.TRANS_ROT270:
            case EV3GraphicsLCD.TRANS_ROT90:
                ow = h;
                oh = w;
                break;
        }
        // Create empty new image
        Image newImage = createLegoImage(ow, oh);
        GraphicsLCD g = newImage.getGraphics();
        g.drawRegion(image, x, y, w, h, transform, 0, 0, 0);
        return newImage;
    }

    /**
     * Return the width of the image.
     * @return Image width
     */
    public int getWidth()
    {
        return width;
    }

    /**
     * return the height of the image.
     * @return image height
     */
    public int getHeight()
    {
        return height;


    }

    /**
     * Return the byte array used to hold the image data.
     * <br>Note: This is a non standard method.
     * @return The image byte array.
     */
    public byte[] getData()
    {
        return data;
    }

    /**
     * Returns a graphics object that can be used to draw to the image.
     * @return graphics object.
     * @see EV3GraphicsLCD
     */
    public EV3GraphicsLCD getGraphics()
    {
        return new EV3GraphicsLCD(data, width, height);
    }
}