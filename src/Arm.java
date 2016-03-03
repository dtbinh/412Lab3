import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import lejos.utility.Matrix;
import lejos.utility.Delay;

/*
 * 2DOF arm class
 */
public class Arm {
	
	EV3LargeRegulatedMotor j1;
	EV3LargeRegulatedMotor j2;
	public static TrackerReader tracker;
	
	double l1;
	double l2;
	
	public Arm() {
		this.j1 = new EV3LargeRegulatedMotor(MotorPort.D);
		this.j2 = new EV3LargeRegulatedMotor(MotorPort.A);
		
		j1.setSpeed(50);
		j2.setSpeed(50);
		
		this.l2 = 8.7;
		this.l1 = 11.2;
	}
	
	public void goToAngle(double theta1, double theta2) {
		this.j1.rotateTo((int)theta1, true);
		this.j2.rotateTo((int)theta2);
		//forwardKinematics(theta1,theta2);
	}
	
	/*
	 * Forward kinematics 
	 */
	public double[] forwardKinematics(double theta1, double theta2){
		double radians1= theta1 * Math.PI/180;
		double radians2 = theta2 * Math.PI/180;
		double x = this.l1*Math.cos(radians1) + this.l2*Math.cos(radians1 + radians2);
		double y = this.l1*Math.sin(radians1) + this.l2*Math.sin(radians1 + radians2);
		
		System.out.printf("x: %.2f \n",x);
		System.out.printf("y: %.2f \n",y);
		double[] pos = {x,y};
		return pos;
	}
	
	/*
	 * Waits for button to be pressed and returns the point the 
	 * arm end effector is currently at .
	 */
	public double[] getPoint(){
		double[] point = {0,0};
		
		this.j1.flt();
		this.j2.flt();
		Button.waitForAnyPress();
		point = forwardKinematics(j1.getTachoCount(),j2.getTachoCount());
		
		return point;
	}
	
	/*
	 * Measures the distance between two points selected by moving 
	 * arm and pressing EV3 brick button
	 */
	public void measureDistance(){
		double[] pos1, pos2 = {0,0};
		double distance;
		
		//this.j1.rotateTo(90, true);
		//this.j2.rotateTo(0);
		
		System.out.println("Pick point \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 2nd point \n and press button");
		pos2 = getPoint();
		
		// d = sqrt(y2-y1)^2 + (x2-x1)^2)
		distance = Math.sqrt(Math.pow((pos2[0]-pos1[0]),2) + Math.pow((pos2[1]-pos1[1]),2));
		
		System.out.printf("p1: (%.1f,%.1f) \n",pos1[0],pos1[1]);
		System.out.printf("p2: (%.1f,%.1f) \n",pos2[0],pos2[1]);
		System.out.printf("distance: %.1f \n",distance);
		
		Button.waitForAnyPress();
	}
	
	/*
	 * Numerical solution to inverse kinematics, uses newtons method with 15 iterations
	 */
	public double[][] invKinematics(double x, double y){
		Random r = new Random((long) 0.54879);
		double[][] angles = {{Math.toRadians(j1.getTachoCount() + r.nextDouble()/100)},{Math.toRadians(j2.getTachoCount() + r.nextDouble()/100)}};
		double[] pos = {x,y};
		Matrix angleMat = new Matrix(angles);
		double Jx1,Jx2,Jy1,Jy2;

		for(int i = 0; i < 15;i++){
			
			Jx1 = -1*this.l1*Math.sin(angleMat.get(0,0)) - this.l2*Math.sin(angleMat.get(0,0) + angleMat.get(1, 0));
			Jx2 = -1*this.l2*Math.sin(angleMat.get(0,0) + angleMat.get(1, 0));
			
			Jy1 = this.l1*Math.cos(angleMat.get(0,0)) - this.l2*Math.cos(angleMat.get(0,0) + angleMat.get(1, 0));
			Jy2 = this.l2*Math.cos(angleMat.get(0,0) + angleMat.get(1, 0));
			
			double[][] array = {{Jx1,Jx2},{Jy1,Jy2}};
			Matrix J = new Matrix(array);
			
			Matrix s = J.solve(f(angleMat, pos));
			angleMat.plusEquals(s);
		}
		angles = angleMat.getArrayCopy();
		
		angles[0][0] = angles[0][0] % (2*Math.PI);
		angles[1][0] = angles[1][0] % (2*Math.PI);
		
		return angles;
	}
	/*
	 * Analytic solution to inverse kinematics 
	 */
	public double[][] invKinematics2(double x, double y){
		double[][] angles = {{0},{0}};
		
		double D = (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(this.l1, 2) - Math.pow(this.l2, 2)) / (2 * this.l1*this.l2);
		
		double z = Math.sqrt(1-Math.pow(D, 2));
		
		double theta2 = Math.atan2(z,D);
		
		double theta1 =Math.atan(y/x) - Math.atan(this.l2 * Math.sin(theta2) /(this.l1 + this.l2*Math.cos(theta2)));
		
		angles[0][0] = theta1;
		angles[1][0] = theta2;
		
		return angles;
	}
	
	/*
	 * Using invkinematics calculates angles to get to a point 
	 * and moves to that point
	 */
	public void goToPoint(double x, double y){
		double[][] angles = invKinematics2(x,y);
		
		System.out.printf("theta1: %.2f \n",Math.toDegrees(angles[0][0]));
		System.out.printf("theta2: %.2f \n",Math.toDegrees(angles[1][0]));

		this.goToAngle(Math.toDegrees(angles[0][0]), Math.toDegrees(angles[1][0]));
		
	}
	
	/*
	 * Calculates f for newtons method
	 */
	public Matrix f(Matrix angles, double[] pos){
		double fx = this.l1*Math.cos(angles.get(0, 0)) + this.l2*Math.cos(angles.get(0, 0) + angles.get(1, 0)) - pos[0];
		double fy = this.l1*Math.sin(angles.get(0, 0)) + this.l2*Math.sin(angles.get(0, 0) + angles.get(1, 0)) - pos[1];
		double[][] f = {{-fx},{-fy}};
		
		return new Matrix(f);
	}
	
	/*
	 * Draws a line given a distance, start point and angle
	 */
	public void angleDistLine(double x, double y, double theta, double dist) {
		double x2 = dist*Math.cos(theta);
		double y2 = dist*Math.sin(theta);
		
		this.straightLine(x, y, x2, y2);
	}
	/*
	 * Measures angle given three points picked by user
	 */
	public void measureAngle(){
		double[] pos1, pos2, pos3 = {0,0};
		double distance, m1, m2;
		
		System.out.println(j1.getTachoCount());
		System.out.println(j2.getTachoCount());
		
		System.out.println("Pick intersection \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 1st line \n and press button");
		pos2 = getPoint();
		System.out.println("Pick 2nd line \n and press button");
		pos3 = getPoint();
		
		double u[] = {pos2[0] - pos1[0],pos2[1] - pos1[1]};
		double v[] = {pos3[0] - pos1[0],pos3[1] - pos1[1]};
		
		double ul = Math.sqrt(Math.pow(u[0],2) + Math.pow(u[1], 2));
		double vl = Math.sqrt(Math.pow(v[0],2) + Math.pow(v[1], 2));
		
		double angle = Math.acos(((u[0]*v[0])+u[1]*v[1])/(ul*vl));	
		
		System.out.printf("p1: (%.1f,%.1f) \n",pos1[0],pos1[1]);
		System.out.printf("p2: (%.1f,%.1f) \n",pos2[0],pos2[1]);
		System.out.printf("p3: (%.1f,%.1f) \n",pos3[0],pos3[1]);
		System.out.printf("angle: %.1f \n",Math.toDegrees(angle));
		Button.waitForAnyPress();
	}
	
	/*
	 * Finds midpoint by allowing user to move arm to two 
	 * points, calculates the midpoint, then uses inverse 
	 * kinematics to calculate angles to get to that point 
	 */
	public void findMidPoint(){
		double[] pos1, pos2 = {0,0};
		double distance;
		
		System.out.println("Pick poinxt \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 2nd point \n and press button");
		pos2 = getPoint();
		
		double x = (pos1[0] + pos2[0])/2;
		double y = (pos1[1] + pos2[1])/2;
		
		this.goToPoint(x, y);		
		
		System.out.printf("p1: (%.1f,%.1f) \n",pos1[0],pos1[1]);
		System.out.printf("p2: (%.1f,%.1f) \n",pos2[0],pos2[1]);
		System.out.printf("mid: (%.1f,%.1f) \n",x,y);
		
		Button.waitForAnyPress();
	}
	
	/*
	 * Draw straight line between two points 
	 */
	public void straightLine(double x1, double y1, double x2, double y2) {
		this.goToPoint(x1,y1);
		int intPoints = 10;
		int y = 0;
		
		double deltaX;
		double deltaY;
		
		if (x1 == x2) {
			deltaX = 0;
			deltaY = (y2-y1)/intPoints;
		} else {
			double slope = (y2-y1)/(x2-x1);
			deltaX = (x2-x1)/intPoints;
			deltaY = slope*deltaX;
		}
		
		Double currY = y1;
		Double currX = x1;
		
		while (y < intPoints) {
			currX= deltaX;
			currY += deltaY;
			
			System.out.println("X: " + currX.toString() + " Y: " + currY.toString());
			this.goToPoint(currX, currY);
			y += 1;
		}
	}
	
	
	public Matrix initJacobian(){
		double [][] jacob = {{0,0},{0,0}};
		double uStart = tracker.x;
		double vStart = tracker.y;
		
		this.goToAngle(45, 0);

		double uEnd = tracker.x;
		double vEnd = tracker.y;
		
		jacob[0][0] = (uEnd - uStart) / 45;
		jacob[1][0] = (vEnd - vStart) / 45;
		
		this.goToAngle(0, 0);
		
		uStart = tracker.x;
		vStart = tracker.y;
		
		this.goToAngle(0, 45);
		
		uEnd = tracker.x;
		vEnd = tracker.y;
		
		jacob[0][1] = (uEnd - uStart) / 45;
		jacob[1][1] = (vEnd - vStart) / 45;
		
		return new Matrix(jacob);
	}
	
	public static void main(String[] args) {
		Arm a = new Arm();
		tracker = new TrackerReader();
		tracker.start();
		Matrix J = a.initJacobian();
		J.print(System.out);
		Button.waitForAnyPress();
	
	}
}

