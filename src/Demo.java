//import lejos.pc.comm.NXTConnector;
//import lejos.nxt.Motor;
//import lejos.util.Delay;

public class Demo {
	public static TrackerReader tracker;


	public static void main (String[] args) {
		tracker = new TrackerReader();
		tracker.start();
		while (true) {
			//Delay.msDelay(1000);
			try {
				Thread.sleep(1000);         //1000 milliseconds is one second.
			} catch(InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
			System.out.println(tracker.x + " " + tracker.y);
		}

	}
}
