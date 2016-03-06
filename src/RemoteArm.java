import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Matrix;
import lejos.utility.Delay;

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
		double angleMove = 10.0;
		double [][] jacob = {{0,0},{0,0}};
		double uStart = tracker.x;
		double vStart = tracker.y;
		
		this.goToAngle(angleMove, 0);

		double uEnd = tracker.x;
		double vEnd = tracker.y;
		
		jacob[0][0] = (uEnd - uStart) / angleMove;
		jacob[1][0] = (vEnd - vStart) / angleMove;
		
		this.goToAngle(0, 0);
		
		uStart = tracker.x;
		vStart = tracker.y;
		
		this.goToAngle(0, angleMove);
		
		uEnd = tracker.x;
		vEnd = tracker.y;
		
		jacob[0][1] = (uEnd - uStart) / angleMove;
		jacob[1][1] = (vEnd - vStart) / angleMove;
		
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
		int MAX_INT = 50;
		Matrix error, angles, J, Jupdate, deltax;
		double currentx, currenty, e, eold, threshold;
		
		J = initJacobian();
		error = new Matrix(2,1);
		angles = new Matrix(2, 1);
		
		currentx = tracker.x;
		currenty = tracker.y;
		
		error.set(0, 0, tracker.targetx - currentx);
		error.set(1, 0, tracker.targety - currenty);
		
		try {
			angles.set(0, 0, m1.getTachoCount());
			angles.set(1, 0, m2.getTachoCount());
		} catch (RemoteException e1) {
			// TODO Auto-generated catch block
			this.closePorts();
			e1.printStackTrace();
		}
		
		//angles = J.inverse().times(error);
		
		
		e = error.normF();
		threshold = 20;
		
		try {
			for (int i = 1; i<=MAX_INT && e > threshold; i++) {
				currentx = tracker.x;
				currenty = tracker.y;
				
				deltax = J.inverse().times(error);
				angles.plusEquals(deltax.times(-0.01));
				double theta1 = Math.toDegrees(angles.get(0, 0)) % 360;
				double theta2 = Math.toDegrees(angles.get(1, 0)) % 360;
				this.goToAngle(theta1, theta2);

				error.set(0, 0, tracker.targetx - tracker.x);
				error.set(1, 0, tracker.targety - tracker.y);
				
				eold = e;
				e = error.normF();
				
				if (e > 10+eold) {
					System.out.println("Broyden Update");
					Matrix deltay = new Matrix(2,1);
					deltay.set(0, 0, tracker.x - currentx);
					deltay.set(1, 0, tracker.y - currenty);
					
					J.plusEquals((deltay.minus(J.times(deltax)).times(deltax.transpose())).times(1/deltax.transpose().times(deltax).get(0, 0)));
				}
			}
		} catch (Exception exc) {
			this.closePorts();
			exc.printStackTrace();
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
		//System.out.println(tracker.targetx);
		//System.out.println(tracker.targety);
		
		//ra.J = ra.initJacobian();
		//ra.J.print(System.out);
		
		//while (true) {
			//Button.waitForAnyPress();
			ra.moveToTarget();
		//}
		
		ra.closePorts();
		
	}

}
