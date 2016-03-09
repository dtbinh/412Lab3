
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import javax.swing.JFrame;
import javax.swing.JPanel;

import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RMISampleProvider;
import lejos.remote.ev3.RemoteEV3;
import lejos.robotics.SampleProvider;
import lejos.utility.Matrix;
import lejos.utility.Delay;

public class RemoteArm2 extends JPanel{
	public static TrackerReader tracker;
	RMIRegulatedMotor m1;
	RMIRegulatedMotor m2;
	SampleProvider sp;
	EV3UltrasonicSensor dSensor;
	Matrix J;
	boolean ESC;
	float sample[] = {0};


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
		double angleMove = 25.0;
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

	public RemoteArm2() {
		try {
			RemoteEV3 brick = new RemoteEV3("10.0.1.1");
			m1 = brick.createRegulatedMotor("A", 'L');
			m2 = brick.createRegulatedMotor("D", 'L');
			m1.setSpeed(50);
			m2.setSpeed(50);
			
			dSensor = new EV3UltrasonicSensor(brick.getPort(SensorPort.S1.getName()));
			sp = dSensor.getDistanceMode();
			sample = new float[1];
			//sp = brick.createSampleProvider("S1", "lejos.hardware.sensor.NXTUltrasonicSensor", "Distance");
			ESC = false;
			MouseListener l = new MyMouseListener();
			addMouseListener(l);
			setFocusable(true);
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

	public void moveToTarget(double x, double y) {
		int MAX_INT = 10000;
		double MAX_ANGLE = 15;
		Matrix error, angles, J, Jupdate, deltax;
		double currentx, currenty, e, eold, threshold;

		//J = initJacobian();
		this.J = initJacobian();
		//this.J.print(System.out);
		
		error = new Matrix(2,1);
		angles = new Matrix(2, 1);

		currentx = tracker.x;
		currenty = tracker.y;

		error.set(0, 0, x - currentx);
		error.set(1, 0, y - currenty);

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
		threshold = 3;

		try {
			while(!(this.ESC)) {
				if(e < threshold){
					break;
				}
				
				sp.fetchSample(sample,0);
				System.out.println(sample[0]);
				if(sample[0] < 0.1){
					continue;
				}
				
				//System.out.println(i);
				currentx = tracker.x;
				currenty = tracker.y;
				
				deltax = this.J.inverse().times(error);
				deltax = deltax.times(0.3);
				
				if (Math.abs(deltax.get(0, 0)) > MAX_ANGLE) {
					deltax.set(0, 0, MAX_ANGLE * Math.signum(deltax.get(0, 0)));
				}
				if (Math.abs(deltax.get(1, 0)) > MAX_ANGLE) {
					deltax.set(1, 0, MAX_ANGLE * Math.signum(deltax.get(1, 0)));
				}
				
				//System.out.println("DeltaX");
				//deltax.print(System.out);
				angles.plusEquals(deltax);
				//double theta1 = Math.toDegrees(angles.get(0, 0)) % 360;
				//double theta2 = Math.toDegrees(angles.get(1, 0)) % 360;
				double theta1 = angles.get(0, 0);
				double theta2 = angles.get(1, 0);
				//System.out.println(theta1);
				//System.out.println(theta2);
				
				
				this.goToAngle(theta1, theta2);

				//error.set(0, 0, x - tracker.x);
				//error.set(1, 0, y - tracker.y);
				error.set(0, 0, tracker.targetx - tracker.x);
				error.set(1, 0, tracker.targety - tracker.y);


				eold = e;
				e = error.normF();

				if (e > 10+eold) {
					System.out.println("Broyden Update");
					Matrix deltay = new Matrix(2,1);
					deltay.set(0, 0, tracker.x - currentx);
					deltay.set(1, 0, tracker.y - currenty);

					Jupdate = (deltay.minus(this.J.times(deltax)).times(deltax.transpose())).times(1/deltax.transpose().times(deltax).get(0, 0));
					Jupdate = Jupdate.times(0.5);
					this.J.plusEquals(Jupdate);
					//this.J.print(System.out);
					//System.out.println("DeltaY");
					//deltay.print(System.out);
				}
			}
			System.out.println("Done");
		} catch (Exception exc) {
			this.closePorts();
			exc.printStackTrace();
		}
	}

	public void pathPlanning(){
		System.out.printf("Target: %f, %f\n", tracker.targetx, tracker.targety);
		System.out.printf("Current: %f, %f\n", tracker.x, tracker.y);
		double slope = (tracker.targety - tracker.y) / (tracker.targetx - tracker.x);

		int n = 10;
		
		this.J = initJacobian();

		double curX = tracker.x;
		double curY = tracker.y;

		double deltaX = (tracker.targetx - tracker.x)/n;
		double deltaY = deltaX*slope;


		for(int i = 0; i < n; i++){
			if(this.ESC){
				break;
			}
			curX = curX + deltaX;
			curY = curY + deltaY;
			
			System.out.printf("%f, %f\n", curX, curY);

			this.moveToTarget(curX, curY);
		}


	}

	public void closePorts(){
		try {
			m1.close();
			m2.close();
			dSensor.close();
		} catch (RemoteException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			this.closePorts();
		}
	}
	

	public class MyMouseListener implements MouseListener{

		@Override
		public void mouseClicked(MouseEvent arg0) {
			ESC = true;
			System.out.println("Break");
			
		}

		@Override
		public void mouseEntered(MouseEvent arg0) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void mouseExited(MouseEvent arg0) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void mousePressed(MouseEvent arg0) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void mouseReleased(MouseEvent arg0) {
			// TODO Auto-generated method stub
			
		}
		
	}

	public static void main(String[] args) {
			// TODO Auto-generated catch block[] args) {
		tracker = new TrackerReader();
		tracker.start();

		Button.waitForAnyPress();
		
		RemoteArm2 ra = new RemoteArm2();
		
		JFrame frame = new JFrame("Click here to escape");
		frame.add(ra);
		frame.setSize(250, 250);
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		
		//System.out.println(tracker.targetx);
		//System.out.println(tracker.targety);

		//ra.J = ra.initJacobian();
		//ra.J.print(System.out);
		//while (true) {
			//Button.waitForAnyPress();
			//ra.pathPlanning();
			//ra.J = initJacobian();
			ra.moveToTarget(tracker.targetx, tracker.targety);
			//ra.goToAngle(0.0, 0.0);
			//ra.closePorts();
			//ra.pathPlanning();
		//}

		ra.closePorts();

	}
}
