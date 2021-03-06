
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
/*
 * Robot are usin Visual servoing with collision detection
 * -Uses RemoteEV3, jframe for kill switch
 * -First start this application then start tracker server (tracker.py)
 */
public class RemoteArm2 extends JPanel{
	public static TrackerReader tracker;
	RMIRegulatedMotor m1;
	RMIRegulatedMotor m2;
	SampleProvider sp;
	EV3UltrasonicSensor dSensor;
	Matrix J;
	boolean ESC;
	float sample[] = {0};

	/*
	 * Rotates arm to specified angles
	 */
	public void goToAngle(double theta1, double theta2) {
		try {
			this.m1.rotateTo((int)theta1);
			this.m2.rotateTo((int)theta2);
		} catch (RemoteException e) {
			this.closePorts();
			e.printStackTrace();
		}
	}

	/*
	 * Initializes Jacobian using orthogonal arm movements and takes
	 * difference in start and end postions 
	 */
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
			this.J = initJacobian();
			//distance sensor init
			dSensor = new EV3UltrasonicSensor(brick.getPort(SensorPort.S1.getName()));
			sp = dSensor.getDistanceMode();
			sample = new float[1];
			ESC = false;
			MouseListener l = new MyMouseListener();
			addMouseListener(l);
			setFocusable(true);
		} catch (RemoteException e) {
			e.printStackTrace();
			this.closePorts();
		} catch (MalformedURLException e) {
			e.printStackTrace();
			this.closePorts();
		} catch (NotBoundException e) {
			e.printStackTrace();
			this.closePorts();
		}
	}

	/*
	 * Moves robot end effector to target using visual servoing 
	 * inludes collision detection
	 */
	public void moveToTarget(double x, double y, boolean OD) {
		int MAX_INT = 10000;
		double MAX_ANGLE = 15;
		Matrix error, angles, J, Jupdate, deltax;
		double currentx, currenty, e, eold, threshold;
		
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
			this.closePorts();
			e1.printStackTrace();
		}

		e = error.normF(); 	
		threshold = 3;

		try {
			while(!(this.ESC)) {
				if(e < threshold){
					break;
				}
				//fetch distance measure and check for collision
				sp.fetchSample(sample,0);
				System.out.println(sample[0]);
				if(sample[0] <= 0.1 && OD){
					//rotate sensor until clear path is found
					while(sample[0] <= 0.1){
						sp.fetchSample(sample,0);
						m1.rotate(15);
					}
					//avoid obstacle
					this.pathPlanning();
					
					System.out.println("Done avoiding");
					//set angles and error to new position
					angles.set(0, 0, m1.getTachoCount());
					angles.set(1, 0, m2.getTachoCount());
					error.set(0, 0, tracker.targetx - tracker.x);
					error.set(1, 0, tracker.targety - tracker.y);
				}

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
				
				angles.plusEquals(deltax);
				double theta1 = angles.get(0, 0);
				double theta2 = angles.get(1, 0);
				
				this.goToAngle(theta1, theta2);

				error.set(0, 0, tracker.targetx - tracker.x);
				error.set(1, 0, tracker.targety - tracker.y);

				eold = e;
				e = error.normF();
				//Broyden update when error becomes too large
				if (e > 10+eold) {
					System.out.println("Broyden Update");
					Matrix deltay = new Matrix(2,1);
					deltay.set(0, 0, tracker.x - currentx);
					deltay.set(1, 0, tracker.y - currenty);

					Jupdate = (deltay.minus(this.J.times(deltax)).times(deltax.transpose())).times(1/deltax.transpose().times(deltax).get(0, 0));
					Jupdate = Jupdate.times(0.5);
					this.J.plusEquals(Jupdate);
				}
			}
			System.out.println("Done");
		} catch (Exception exc) {
			this.closePorts();
			exc.printStackTrace();
		}
	}

	/*
	 * Moves arm towards point in clearpath direction
	 */
	public void pathPlanning(){
		System.out.println("Obstacle detected: start path planning");

		double curX = tracker.x;
		double curY = tracker.y;

		double newX = tracker.x - 20;
		double newY = tracker.y + 15;

		this.moveToTarget(newX, newY, false);
	}
	
	/*
	 * Closes remote ports
	 */
	public void closePorts(){
		try {
			m1.close();
			m2.close();
			dSensor.close();
		} catch (RemoteException e) {
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
			
		}

		@Override
		public void mouseExited(MouseEvent arg0) {
			
		}

		@Override
		public void mousePressed(MouseEvent arg0) {
			
		}

		@Override
		public void mouseReleased(MouseEvent arg0) {
			
		}
	}

	public static void main(String[] args) {
		//start tracker client 
		tracker = new TrackerReader();
		tracker.start();

		Button.waitForAnyPress();
		
		RemoteArm2 ra = new RemoteArm2();
		
		JFrame frame = new JFrame("Click here to escape");
		frame.add(ra);
		frame.setSize(250, 250);
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		ra.moveToTarget(tracker.targetx, tracker.targety, true);

		ra.closePorts();
	}
}
