package rimesime.fu.turtleremote;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.view.Surface;

public class Drive implements SensorEventListener {

	static char KEYCODE_R = 0x43;
	static char KEYCODE_L = 0x44;
	static char KEYCODE_U = 0x41;
	static char KEYCODE_D = 0x42;

	boolean isDriving = false;

	private boolean lockCurrentState = false;
	private int lockedX = 0, lockedY = 0;

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

		int intX = Math.round(floatX * State.varAccuracyMultiplyer);
		int intY = Math.round(floatY * State.varAccuracyMultiplyer);

		if (lockCurrentState) {
			lockCurrentState = false;
			lockedX = intX;
			lockedY = intY;
		}

		intX -= lockedX;
		intY -= lockedY;

		Log.s("Driving... (" + lockedX + "+" + intX + ", " + lockedY + "+"
				+ intY + ")");

		if ((intX == 0) && (intY == 0))
			return;

		while ((intX != 0) && (intY != 0)) {
			if (intX > 0) {
				intX--;
				for (int i = 0; i < State.varCommandMultiplyer; ++i) {
					State.connection.print(KEYCODE_L);
				}
			}
			if (intX < 0) {
				intX++;
				for (int i = 0; i < State.varCommandMultiplyer; ++i) {
					State.connection.print(KEYCODE_R);
				}
			}
			if (intY > 0) {
				intY--;
				for (int i = 0; i < State.varCommandMultiplyer; ++i) {
					State.connection.print(KEYCODE_D);
				}
			}
			if (intY < 0) {
				intY++;
				for (int i = 0; i < State.varCommandMultiplyer; ++i) {
					State.connection.print(KEYCODE_U);
				}
			}
		}

	}

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	public void lockCurrentState() {
		lockCurrentState = true;
	}

}
