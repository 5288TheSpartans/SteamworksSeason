package Accessories;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class VisionCalculator implements ITableListener{
	final int PIXEL_WIDTH = 720;
	final int PIXEL_HEIGHT =  480;
	final int LEEWAY = 14;
	final double VFOV = 55.738846;
	final double kyrelConstant = 761.4573343;
	final double jamesConstant = 0.45;
	double dTarget = 0;
	double aTarget =0;
	//Indexes for found targets
	boolean isfound = false;
	int leftContourIndex;
	int rightContourIndex;
	boolean isleftContourfull;
	boolean isrightContourfull;
	//NetworkTables raw data
	double widths[];
	double heights[];
	double centerYs[];
	double centerXs[];
	//Tested NetworkTables data
	double realAreas[];
	double realWidths[];
	double realHeights[];
	double realCenterYs[];
	double realCenterXs[];
	int indexes[];
	int foundtargets;
	//Probability calculator
	double probabilities[];
	double isPossible[];
	NetworkTable gripTable; 
	boolean isInitialized = false;
	public VisionCalculator(){
	}
	public void Updatedata(){
		System.out.println("Is Vision Running:" + isInitialized);
	/*	if(isInitialized)
		{
			loadData();
			if(centerYs.length> 0 && centerYs[0] != 0)
			{
				calculateProbabilities();
			}
		}
		else
		{
			try
			{
				gripTable = NetworkTable.getTable("GRIP/LowgoalCameraContours");
				gripTable.addTableListener(this);
				isInitialized = true;
			}
			catch(Exception e)
			{
				isInitialized = false;
			}
		}*/
	}
	private void loadData(){
		double[] defaultValue = new double[0];
		//Get all data from the GRIP program
		foundtargets = 0;
		widths = gripTable.getNumberArray("width",defaultValue);
		heights = gripTable.getNumberArray("height",defaultValue);
		centerYs = gripTable.getNumberArray("centerY",defaultValue);
		centerXs = gripTable.getNumberArray("centerX",defaultValue);
		probabilities = new double[widths.length];
		indexes = new int[widths.length];
	}
	public double getDistance(){
		if(isfound)
		{
			return dTarget;
		}
		else{
			return 0;
		}
	}
	public double getAngle(){
		if(isfound)
		{
			return aTarget;
		}
		else{
			return 180;
		}
	}
	public void calculateProbabilities()
	{

		System.out.println("TESTING VISION");
		boolean isPossibleTarget[] = new boolean[probabilities.length];
		if(checkValidity())
		{
			for(int i = 0; i < probabilities.length; i ++)
			{
				System.out.println("i =" + i);
				double contourBottom = (heights[i]/2) + centerYs[i]; 
				if(widths[i] > heights[i] || ((widths[i]/2) + centerXs[i]) >= PIXEL_WIDTH || (centerXs[i] - (widths[i]/2) <= 0) )
				{
					System.out.println("PATH 1");
					isPossibleTarget[i] = false;
					probabilities[i] = 0;
				}
				else if( contourBottom < ((PIXEL_HEIGHT/2)) && contourBottom > ((PIXEL_HEIGHT/2) -LEEWAY)  )
				{

					System.out.println("PATH 2");
					isPossibleTarget[i] = true;
					probabilities[i] = 100;
					foundtargets ++;
					/**Filter to make sure you see at least 2 matching contours, after checking all the contours
					 * 
					 */
					System.out.println("Found viable targets");
				}
				else
				{

					System.out.println("PATH 3");
					isPossibleTarget[i] = false;
					probabilities[i] = 0;
				}
			}
			int reorderingCounter = 0;
			foundtargets = 0;
			System.out.println("COMPLETED VISION PATH");
			for(int i = 0; i < isPossibleTarget.length; i ++ )
			{
				if(isPossibleTarget[i])
				{	
					indexes[reorderingCounter] = i;
					reorderingCounter ++;
					foundtargets ++;
					System.out.println("Foundtarget calculated, index noted");
				}
			}

			if(foundtargets == 2)
			{
				System.out.println("# of Found Contours: 2");
				isfound = false;
				//Sort to find which of the two given contours is left, and which is right.
				if( centerXs[indexes[0]] > centerXs[indexes[1]])
				{

					rightContourIndex = indexes[0];
					leftContourIndex = indexes[1];
					System.out.println("Left Target index : " + leftContourIndex);
					System.out.println("Right Target index : " + rightContourIndex);
					isfound = true;
					System.out.println("Targets Found!");
					if(heights[leftContourIndex]> (heights[rightContourIndex ]/2))
					{

						System.out.println("Left Contour is double the right contour, must do extra processing");
					}
					else if(heights[rightContourIndex ]> (heights[leftContourIndex]/2))
					{
						System.out.println("Right Contour is double the right contour, must do extra processing");
					}
					else
					{
						System.out.println("Contours are relatively close in sizing.");
						//Continue, believing that the contours are found and are fully intact.
						isfound = true;
						dTarget = calculateDistance(heights[leftContourIndex], heights[rightContourIndex]);
						System.out.println("Found Target at Distance of:" + dTarget);
					}

				}
				else if( centerXs[indexes[0]] < centerXs[indexes[1]])
				{
					//locates left and right
					isfound = true;
					rightContourIndex = indexes[1];
					leftContourIndex = indexes[0];
					if(heights[leftContourIndex]> (heights[rightContourIndex ] - 80))
					{
						//Search for a third fitting contour that would add to rightcontour to become a full contour (compensates for missing peg)

					}
					else if(heights[rightContourIndex ]> (heights[leftContourIndex]- 80))
					{
						//Search for a third fitting contour that would add to leftcontour to become a full contour (compensates for missing peg)
					}
					else
					{
						isfound = true;
						dTarget = calculateDistance(heights[leftContourIndex], heights[rightContourIndex]);
					}

				}
				else
				{
					isfound = false;
				}
			}
			else
			{
				isfound = false;
			}
		}
	}
	private boolean checkValidity(){
		if(widths.length == heights.length && widths.length == centerXs.length && widths.length == centerYs.length){
			return true;
		}
		else
		{
			return false;
		}
	}
	//This function returns -1 if there is no most 
	private double calculateAngleDEGREES(double widthL, double heightL, double centerxL, double centeryL, double widthR, double heightR, double centerxR,double centeryR)
	{

		//Just a bunch of math, dont ask questions, believe.

		return 0;
	}
	private double calculateDistance(double heightL, double heightR)
	{
		//Just a bunch of math, dont ask questions.
		double Ld = kyrelConstant*heightL + jamesConstant;
		double Rd = kyrelConstant*heightR + jamesConstant;
		double distance;
		//finds true distance
		distance = (Ld + Rd)/2 ;
		return distance;
	}
	@Override
	public void valueChanged(ITable source, String key, Object value, boolean isNew) {
		// TODO Auto-generated method stub

	}

}
