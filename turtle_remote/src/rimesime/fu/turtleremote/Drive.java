package rimesime.fu.turtleremote;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.view.Surface;

public class Drive implements SensorEventListener {

	static char KEYCODE_R = 'd'; // 0x43;
	static char KEYCODE_L = 'a'; // 0x44;
	static char KEYCODE_U = 'w'; // 0x41;
	static char KEYCODE_D = 's'; // 0x42;

	boolean isDriving = false;

	private boolean lockCurrentState = false;
	private int lockedX = 0, lockedY = 0;
	private int lastX = 0, lastY = 0;

	public void onSensorChanged(SensorEvent event) {
		if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
			return;

		float floatX = 0;
		float floatY = 0;

		switch (Turtle_RemoteActivity.display.getRotation()) {
		case Surface.ROTATION_0:
			floatX = event.values[0];
			floatY = event.values[1];
			break;
		case Surface.ROTATION_90:
			floatX = -event.values[1];
			floatY = event.values[0];
			break;
		case Surface.ROTATION_180:
			floatX = -event.values[0];
			floatY = -event.values[1];
			break;
		case Surface.ROTATION_270:
			floatX = event.values[1];
			floatY = -event.values[0];
			break;
		}

		int intX = Math.round(floatX);
		int intY = Math.round(floatY);

		if (lockCurrentState) {
			lockCurrentState = false;
			lockedX = intX;
			lockedY = intY;
		}

		intX -= lockedX;
		intY -= lockedY;

		if (Math.abs(intX) == Math.abs(intY)) {
			intX = 0;
			intY = 0;
		}

		if (Math.abs(intX) > Math.abs(intY))
			intY = 0;
		else
			intX = 0;

		intX = intX > 0 ? 1 : (intX == 0 ? 0 : -1);
		intY = intY > 0 ? 1 : (intY == 0 ? 0 : -1);

		if (intY == lastY && intX == lastX)
			return;

		lastX = intX;
		lastY = intY;

		Log.s("Driving... (X: " + intX + ", Y: " + intY + ")");

		switch (intX) {
		case +1:
			State.connection.print(KEYCODE_L);
			break;
		case -1:
			State.connection.print(KEYCODE_R);
			break;
		}
		
		switch (intY) {
		case +1:
			State.connection.print(KEYCODE_D);
			break;
		case -1:
			State.connection.print(KEYCODE_U);
			break;
		}

	}

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	public void lockCurrentState() {
		lockCurrentState = true;
	}

}
