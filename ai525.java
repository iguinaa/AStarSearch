/*
   CAP4630 Assignment 1 by Andrew Iguina
   For: Dr. Chuan
   Submitted: 2/18/2016 (one day late)
   
   This program performs A* search to get from a start point to a goal point optimally without intersecting
   obstacles. The map should be included as a command line argument. This program also requires
   Java 8 for the Comparator interface and single lambda expression. This could be edited out in
   future, but it seems more streamlined this way. In a future iteration, I believe I could move
   most if not all of the globals into main and pass them as normal arguments. I have included 
   Node and Comparator<Node> classes which allow for the use of some standard Java data 
   structures and utilities.

*/

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.function.Consumer;

public class ai525
{
	static Point2D goal;
	static Point2D start;
	// To contain:  Our State Space - ( start U goal states )  
	static ArrayList<Line2D> polyLines;
	static ArrayList<Point2D> polyPoints;
	static ArrayList<Polygon> polys;
	
	static PriorityQueue<Node> frontier;
//	static ArrayList<Node> explored;
	
	public static void main(String[] args) throws FileNotFoundException
	{

		File points = new File(args[0]); 

		if(points == null)
		{
			throw new FileNotFoundException();
		}
		Scanner input = new Scanner(points);
		String nextLine = new String();
		Scanner stringParser;
		polyLines = new ArrayList<Line2D>();
		polyPoints = new ArrayList<Point2D>();
		polys = new ArrayList<Polygon>();
		
		ArrayList<Point2D> polyBounds = new ArrayList<Point2D>();
		int numLoops = 0;

		while(input.hasNextLine())
		{	
			polyBounds.clear();
			nextLine = input.nextLine();

			stringParser = new Scanner(nextLine);
			stringParser.useDelimiter("\\s*,\\s*|\\s*;\\s*");

			if(numLoops == 0)
			{
				start = createPointFromInput(stringParser);
//				System.out.println("start: " + start);
			}
			else if(numLoops == 1)
			{
				goal = createPointFromInput(stringParser);
//				System.out.println("goal: " + goal);
			}
			else
			{
				Polygon poly = null;
				Point2D polygonStart;
				int[] xcoords;
				int[] ycoords;
				Point2D from;
				Point2D to;
				
				polygonStart = createPointFromInput(stringParser);
				polyPoints.add(polygonStart);
				polyBounds.add(polygonStart);
				from = polygonStart;
				
				while(stringParser.hasNextInt())
				{
					to = createPointFromInput(stringParser);
					polyPoints.add(to);
					polyBounds.add(to);
					//make a line from "from" to "to"
					polyLines.add(new Line2D.Double(from,to));
					from = to;
				}
				to = polygonStart;
				polyLines.add(new Line2D.Double(from,to));
				xcoords = new int[polyBounds.size()];
				ycoords = new int[polyBounds.size()];
				for(int i = 0; i < polyBounds.size(); i++)
				{
					xcoords[i] = (int)(polyBounds.get(i).getX());
					ycoords[i] = (int)(polyBounds.get(i).getY());
				}
				poly = new Polygon(xcoords, ycoords, polyBounds.size());
				polys.add(poly);
			}
			numLoops++;
		}
		CompareNode comparator = new CompareNode(goal);
		frontier = new PriorityQueue<Node>(comparator);
		// A list of coordinates to be printed in sequence including start and goal state. (Forms path to goal)
		ArrayList<Node> solution = AStar();	// store from goal to start, print backwards
		ArrayList<Point2D> finalAnswer = new ArrayList<Point2D>();
		for(int i=solution.size()-1; i >= 0; i--)
		{
//			System.out.println(solution.get(i));
			finalAnswer.add(solution.get(i).getState());
		}
		// Removing extra "Stops" along path. Ex: [(0,0),(0,1)]+[(0,1),(0,2)] ---> [(0,0),(0,2)]
		for(int i=0; i < finalAnswer.size()-2; i++)
		{
			if( compareSlopes(finalAnswer.get(i), finalAnswer.get(i+1), finalAnswer.get(i+2))  )
			{
				finalAnswer.remove(i+1);
				i--;
			}
		}
		for(int i = 0; i < finalAnswer.size(); i++)
		{
			System.out.println("(" + finalAnswer.get(i).getX() + ", " + finalAnswer.get(i).getY() + ")");
		}
	}
	/** This compares the slope of two lines (a,b) and (b,c)*/
	private static boolean compareSlopes(Point2D a, Point2D b, Point2D c) 
	{
		double numerator = b.getY() - a.getY();
		double denominator = b.getX() - a.getX();
		
		boolean sameSlope = true;
		
		if(numerator != (c.getY() - b.getY()) )
			sameSlope = false;
		if(numerator != (c.getY() - b.getY()) )
			sameSlope = false;
		
		return sameSlope;
	}
	
   /** This implements A* using many different my classes and some Java standard priority queue */
	public static ArrayList<Node> AStar()
	{
		int debugNumIterations = 0;
		Node root = new Node();
		root.setState(start);
		frontier.add(root);
		ArrayList<Node> explored = new ArrayList<Node>();
		Node parent;
		do
		{
			if(frontier.isEmpty())
				return null;
			parent = frontier.remove();
			if(parent.getState().equals(goal))
			{
				// return solution
				Node solutionVert = parent;
				ArrayList<Node> solutionSet = new ArrayList<Node>();
				while(solutionVert != null)
				{
					solutionSet.add(solutionVert);
					solutionVert = solutionVert.getParent();
				}
				return solutionSet;
				
			}
			else
			{
				parent.setExplored(true);
				explored.add(parent);
				//System.out.println("\nparent: " + parent);
				ArrayList<Point2D> children = ACTIONS(parent.getState());
				for(Point2D pt : children)
				{
					Node child = new Node(pt, parent);
					boolean onFront = frontier.contains(child);
					if(onFront)
					{
						frontier.forEach(new Consumer<Node>() 
						{
							public void accept(Node n)
							{
								int result = 0;
								result = child.compareTo(n);
								if(result == -1)
								{
									n.setExplored(true);
									n = child;
//									System.out.println("swapped children nodes");
								}

							}
						});
					}
					else
					{
						if( !explored.contains(child) )
						{
							frontier.add(child);
						}
					}
              
				}
//            ArrayList<Node> dbgFrontier = new ArrayList<Node>();
  //          dbgFrontier.addAll(0, frontier);
    //        System.out.println("children: ");
      //      for(Node p : dbgFrontier)
        //    {
//                 System.out.println(p);
  //          }
			}
			
			debugNumIterations++;
			if(debugNumIterations > 100)
				System.out.println("Hit 100 iterations, Infinite Loop?");
		} while(true);
	}
	
   //
	/** Returns if theres an intersection or null if no intersection */
	public static boolean intersection(Line2D l1, Line2D l2)
	{
		boolean intersect = false;
		
		if(l1.intersectsLine(l2))
		{
			//check if start or end points are the same
			// return not intersecting iff exactly 1 start/end pt on line 1 == 1 start/end pt on line 2
			boolean a1a2 = l1.getP1().equals(l2.getP1());
			boolean a1b2 = l1.getP1().equals(l2.getP2());
			boolean b1a2 = l1.getP2().equals(l2.getP1());
			boolean b1b2 = l1.getP2().equals(l2.getP2());
			
			// true iff (1 and only 1 is true)
			if( (a1a2 && b1b2) || (a1b2 && b1a2) )
				intersect = false;
			else if( ( a1a2 || a1b2 || b1a2 || b1b2) && 
					!( (a1a2 && a1b2) || (a1b2 && b1a2) ||(b1a2 && b1b2) ||(b1b2 && a1a2) ) )
				intersect = false;
			else
				intersect = true;
			
		}
		return intersect;
	}
	
	/** */
	public static boolean isContained(Line2D line)
	{
		boolean contained = false;
		Point2D midpoint = new Point2D.Double( ((line.getX1() + line.getX2()) / 2), ((line.getY1() + line.getY2()) / 2) );
		for(Polygon p : polys)
		{
			if(p.contains(midpoint))
				contained = true;
		}
		return contained;
	}
	
	/** Creates point from input, modularized for error checking */
	static Point2D createPointFromInput(Scanner input)
	{
		Point2D point = null;
		double x = 0.0;
		double y = 0.0;
		
		for(int i = 0; i < 2; i++)
		{
	
			if(input.hasNextInt())
			{
				//System.out.println("test hasNextInt");
				if( i == 0 )
					x = input.nextInt();
				else
					y = input.nextInt();
			}
			else
			{
				System.out.println("error in createPointFromInput, no 2 ints left in input stream");
				System.exit(2);
			}
		}	
		
		point = new Point2D.Double(x,y);
//		System.out.println(point + "\n");
		return point;
	}
	
	/** This method returns a list of valid actions from any node "from" to any node "to" */
	static ArrayList<Point2D> ACTIONS(Point2D from)
	{
		ArrayList<Point2D> actions = new ArrayList<Point2D>();
		Line2D l;
		boolean isValid = true;

		//Check path to polyverts and goal
		for(int i = 0; i <= polyPoints.size(); i++)
		{
			Point2D to;
			isValid = true;
			
			if(i==polyPoints.size())
				to = goal;
			else
				to = polyPoints.get(i);
			
			l = new Line2D.Double(from, to);
			
			boolean isABorder = false;
			for(Line2D border : polyLines)
			{
				if( (l.getP1().equals(border.getP1()) && l.getP2().equals(border.getP2())) || 
						(l.getP1().equals(border.getP2()) && l.getP2().equals(border.getP1())) )
					isABorder = true;
			}
			
			if(isABorder)
			{
				actions.add(to);
			}
			else
			{
				for(int j = 0; j < polyLines.size(); j++)
				{
					if( intersection(l, polyLines.get(j)) )
						isValid = false;
				}

				if(isValid && !isContained(l))
					actions.add(to);
			}
		}
		return actions;
	}

	/** This method calculates the entire f(node) */
	public double costFunction(double costToThisNode, Point2D currentNode) 
	{
		double totalCost = 0.0;
		totalCost = costToThisNode;

		double SLD = currentNode.distance(goal); // SLD == straight line distance
		totalCost += SLD;

		return totalCost;
	}

	

}



class Node implements Comparable<Node>
{
	private double costToHere; // PathCost
	private boolean isExpanded; // == isExplored
	private Point2D state;
	private Node parent;
	
	public Node()
	{
		this.state = new Point2D.Double(0,0);
		isExpanded = false;
		costToHere = 0;
		parent = null;
	}
	public Node(Point2D state, Node parent)
	{
		this.state = state;
		isExpanded = false; 
		this.parent = parent;
		costToHere = calcGx();
	}
	
	public double calcGx()
	{
		return ( parent.getCostToHere() + Node.SLD(this.getState(), parent.getState()) );
	}
	
	/** Straight Line Distance from a to b, separately denoted for extra clarity */
	public static double SLD (Point2D a, Point2D b)
	{
		return a.distance(b);
	}
	
	/** This method calculates the entire f(node) */
	public double getTotalCost(Point2D goal) 
	{
		double totalCost = 0.0;
		totalCost = costToHere + Node.SLD(this.getState(), goal);
		return totalCost;
	}
	
	public Node getParent()
	{
		return this.parent;
	}
	
	public void setParent(Node parent)
	{
		this.parent = parent;
	}
	
	/**
	 * @return the costToHere
	 */
	public double getCostToHere() {
		return costToHere;
	}
	/**
	 * @param costToHere the costToHere to set
	 */
	public void setCostToHere(double costToHere) {
		this.costToHere = costToHere;
	}
	/**
	 * @return the isExplored
	 */
	public boolean isExpanded() {
		return isExpanded;
	}
	/**
	 * @param isExplored the isExplored to set
	 */
	public void setExplored(boolean isExpanded) {
		this.isExpanded= isExpanded;
	}
	/**
	 * @return the state
	 */
	public Point2D getState() {
		return state;
	}
	/**
	 * @param state the state to set
	 */
	public void setState(Point2D state) {
		this.state = state;
	}
	
	@Override
	public int compareTo(Node n) 
	{	
		if(this.equals(n))
		{
			if(this.getCostToHere() < n.getCostToHere())
				return -1;
			else
				return 1;
		}
		else
			return 0;
	}
	
	@Override
	public boolean equals(Object o)
	{
		if(o instanceof Node)
		{
			if( this.state.equals( ((Node)o).getState() ) )
				return true;
		}
		return false;
	}
	
	@Override
	public String toString()
	{
		String s = new String(this.state + 
				" totalCost: " + String.format("%.4f",getTotalCost(CompareNode.goal)) + 
				"  g(x)= " + String.format("%.4f", this.costToHere) + 
				"  h(x)= " + String.format("%.4f",this.SLD(state,  CompareNode.goal)) );
		return s;
	}
}

class CompareNode implements Comparator<Node>
{
	public static Point2D goal;
	public CompareNode(Point2D goal)
	{
		this.goal = goal;
	}
	
	@Override
	public int compare(Node n1, Node n2) 
	{
		if(n1.getTotalCost(goal) <= n2.getTotalCost(goal))
			return -1;
		else
			return 1;
	}
	
}



	/*
	 * 
//		System.out.println("ACTIONS FROM START");
//		for(Point2D p : ACTIONS(start))
//		{
//			System.out.println("action: " + p);
//		}
//		
//		
//		Line2D testContains = new Line2D.Double(0,14,9,15);
//		// Line Code
//		// line1: from (2, 6) to (12, 15)
//		Line2D line1 = new Line2D.Double(2, 6, 12, 15);
//
//		// line2: from (1, 9) to (7, 8)
//		Line2D line2 = new Line2D.Double(1, 9, 7, 8);
//
//		// Intersection Code
//		Line2D line5 = new Line2D.Double(1, 3, 2, 4);
//		Line2D line6 = new Line2D.Double(3, 6, 2, 4);
//
//		if(line5.intersectsLine(line6))
//			System.out.println("line5 and line6 have an intersection.");
//		else
//			System.out.println("line5 and line6 have no intersections.");
//		
//		if(isContained(testContains))
//			System.out.println("testContains is contained by a poly");

		/* Note: if line5 represents the robot's next traveling path and 
      	line6 is an edge of a polygonal obstacle, the robot is able to travel
      	from its current place (1, 3) to the destination (2, 4) even line5
      	and line6 intersect at (2, 4) 
// input.useDelimiter("\\s*,\\s*|\\s*;\\s*");
//			String test = input.nextLine();
//			System.out.println(test);
//			Scanner testScan = new Scanner(test);
//			testScan.useDelimiter("\\s*,\\s*|\\s*;\\s*");
//			if(testScan.hasNextInt())
//				System.out.println("test hasNextInt");
//			
//			System.out.println(testScan.nextInt());
//			System.out.println(testScan.nextInt());
//		
	 
	 if(line1.intersectsLine(line2))
      		System.out.println("line1 and line2 have an intersection.");
      	else
      		System.out.println("line1 and line2 have no intersections.");
      	
	Line2D line3 = new Line2D.Double(1, 3, 2, 4);
	Line2D line4 = new Line2D.Double(1, 2, 2, 3);
	
	if(line3.intersectsLine(line4))
		System.out.println("line3 and line4 have an intersection.");
	else
		System.out.println("line3 and line4 have no intersections.");
	
	*/
	
	