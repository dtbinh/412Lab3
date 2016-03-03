import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Matrix;

public class RemoteArm {
	public static TrackerReader tracker;
	RMIRegulatedMotor m1;
	RMIRegulatedMotor m2;
	Matrix J;
	
	
	public void goToAngle(double theta1, double theta2) {
		try {
			this.m1.rotateTo((int)theta1);
			this.m2.rotateTo((int)theta2);
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			this.closePorts();
			e.printStackTrace();
		}
	}
	
	public Matrix initJacobian(){
		double [][] jacob = {{0,0},{0,0}};
		double uStart = tracker.x;
		double vStart = tracker.y;
		
		System.out.println(uStart);
		System.out.println(vStart);
		
		this.goToAngle(25, 0);

		double uEnd = tracker.x;
		double vEnd = tracker.y;
		
		jacob[0][0] = (uEnd - uStart) / 45.0;
		jacob[1][0] = (vEnd - vStart) / 45.0;
		
		this.goToAngle(0, 0);
		
		uStart = tracker.x;
		vStart = tracker.y;
		
		this.goToAngle(0, 25);
		
		uEnd = tracker.x;
		vEnd = tracker.y;
		
		jacob[0][1] = (uEnd - uStart) / 45.0;
		jacob[1][1] = (vEnd - vStart) / 45.0;
		
		this.goToAngle(0, 0);
		
		return new Matrix(jacob);
	}

	public RemoteArm() {
		try {
			RemoteEV3 brick = new RemoteEV3("10.0.1.1");
			m1 = brick.createRegulatedMotor("A", 'L');
			m2 = brick.createRegulatedMotor("D", 'L');
			m1.setSpeed(50);
			m2.setSpeed(50);
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			this.closePorts();
		} catch (MalformedURLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			this.closePorts();
		} catch (NotBoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			this.closePorts();
		}
		
		
	}
	
	public void moveToTarget() {
		double [][] t = {{tracker.targetx},{tracker.targety}};
		Matrix target = new Matrix(t);
		
		Matrix angles;
		
		try {
			double [][] angl = {{m1.getTachoCount()}, {m2.getTachoCount()}};
			angles = new Matrix(angl);
			while (true) {
				double [][] curr = {{tracker.x}, {tracker.y}};
				Matrix current = new Matrix(curr);
			
				Matrix deltaX = J.solve(target.minus(current));
				angles = angles.plus(deltaX);
				double theta1 = Math.toDegrees(angles.get(0, 0));
				double theta2 = Math.toDegrees(angles.get(1, 0));
				this.goToAngle(theta1, theta2);
			}
		} catch (RemoteException e) {
		// TODO Auto-generated catch block
			this.closePorts();
			e.printStackTrace();
		}
	}
	
	public void closePorts(){
		try {
			m1.close();
			m2.close();
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			this.closePorts();
		}
	}
	
	public static void main(String[] args) {
		tracker = new TrackerReader();
		tracker.start();
		
		Button.waitForAnyPress();
		
		RemoteArm ra = new RemoteArm();
		
		ra.J = ra.initJacobian();
		
		ra.moveToTarget();
		
		ra.closePorts();
		
	}

}
