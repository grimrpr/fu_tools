package rimesime.fu.turtleremote;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.Properties;
import java.util.concurrent.LinkedBlockingQueue;

import android.app.Activity;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnKeyListener;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.jcraft.jsch.Channel;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class Turtle_remoteActivity extends Activity implements
SensorEventListener {

	static String LOG_TAG = "TurtleRemote";

	static float INITIAL_ACCURACY = 1.0f;
	static int INITIAL_SPEED = 1;
	static boolean INITIAL_AUTOCONNECT = false;

	static char KEYCODE_R = 0x43;
	static char KEYCODE_L = 0x44;
	static char KEYCODE_U = 0x41;
	static char KEYCODE_D = 0x42;

	private SharedPreferences prefs;
	static private Properties config = new Properties();
	static {
		config.put("StrictHostKeyChecking", "no");
	}

	private EditText host;
	private EditText password;
	private ScrollView logScroll;
	private LinkedBlockingQueue<String> logQueue = new LinkedBlockingQueue<String>();
	private TextView log;
	private TextView status;
	private Button connectButton;
	private EditText accuracy;
	private EditText speed;
	private ToggleButton autoConnect;
	private Button driveButton;

	private boolean driving = false;

	private SensorManager mSensorManager;
	private Sensor mAccelerometer;
	private WindowManager mWindowManager;
	private Display mDisplay;

	private JSch ssh = new JSch();
	private Session session;
	private Channel channel;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.main);

		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		mAccelerometer = mSensorManager
		.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		mWindowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
		mDisplay = mWindowManager.getDefaultDisplay();

		host = (EditText) findViewById(R.id.host);
		password = (EditText) findViewById(R.id.password);
		logScroll = (ScrollView) findViewById(R.id.logScroll);
		log = (TextView) findViewById(R.id.log);
		status = (TextView) findViewById(R.id.status);
		connectButton = (Button) findViewById(R.id.connect);
		accuracy = (EditText) findViewById(R.id.accuracy);
		speed = (EditText) findViewById(R.id.speed);
		autoConnect = (ToggleButton) findViewById(R.id.autoConnect);
		driveButton = (Button) findViewById(R.id.drive);

		host.setOnKeyListener(new OnKeyListener() {
			@Override
			public boolean onKey(View v, int keyCode, KeyEvent event) {
				doInputStateChange();
				return false;
			}
		});

		autoConnect.setOnCheckedChangeListener(new OnCheckedChangeListener() {
			@Override
			public void onCheckedChanged(CompoundButton buttonView,
					boolean isChecked) {
				SharedPreferences.Editor editor = prefs.edit();
				editor.putBoolean("autoConnect", isChecked);
				editor.commit();
			}
		});

		speed.setOnKeyListener(new OnKeyListener() {
			@Override
			public boolean onKey(View v, int keyCode, KeyEvent event) {
				int i = -1;

				try {
					i = Integer.parseInt(speed.getText().toString());
					speed.setBackgroundColor(Color.WHITE);
				} catch (NumberFormatException e) {
					speed.setBackgroundColor(Color.RED);
					Toast.makeText(v.getContext(), "Must be Integer", 3000);
				}

				SharedPreferences.Editor editor = prefs.edit();
				editor.putInt("speed", i);
				editor.commit();
				return false;
			}
		});

		accuracy.setOnKeyListener(new OnKeyListener() {
			@Override
			public boolean onKey(View v, int keyCode, KeyEvent event) {
				float i = -1f;

				try {
					i = Float.parseFloat(accuracy.getText().toString());
					accuracy.setBackgroundColor(Color.WHITE);
				} catch (NumberFormatException e) {
					accuracy.setBackgroundColor(Color.RED);
					Toast.makeText(v.getContext(), "Must be Float", 3000);
				}

				SharedPreferences.Editor editor = prefs.edit();
				editor.putFloat("accuracy", i);
				editor.commit();
				return false;
			}
		});

		connectButton.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View v) {
				connect();
			}
		});

		driveButton.setOnTouchListener(new OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {

				driveState(event);

				// false: event "not handled", true: button would
				// not look like it got pressed
				return false;

			}
		});

		prefs = getPreferences(MODE_PRIVATE);
		host.setText(prefs.getString("host", ""));
		password.setText(prefs.getString("password", ""));
		speed.setText(new Integer(prefs.getInt("speed", INITIAL_SPEED))
		.toString());
		accuracy.setText(new Float(prefs.getFloat("accuracy", INITIAL_ACCURACY))
		.toString());
		autoConnect.setChecked(prefs.getBoolean("autoConnect",
				INITIAL_AUTOCONNECT));
		doInputStateChange();

		if (autoConnect.isChecked())
			connect();

	}

	private void connect() {
		setInputEnabled(false);
		log("Connecting...");

		SharedPreferences.Editor editor = prefs.edit();
		editor.putString("host", host.getText().toString());
		editor.putString("password", password.getText().toString());
		editor.commit();

		ssh = new JSch();
		String strHost = host.getText().toString();
		String user = strHost.substring(0, strHost.indexOf('@'));
		strHost = strHost.substring(strHost.indexOf('@') + 1);

		try {
			session = ssh.getSession(user, strHost, 22);
			session.setConfig(config);
			session.setPassword(password.getText().toString());
			session.connect();
			channel = session.openChannel("shell");
			channel.setInputStream(null);
			channel.connect(5000);

			log("Connected! Start driving!");
			setDrivingEnabled(true);

			final InputStream is = channel.getInputStream();
			// final LinkedBlockingQueue<String> waitLock1 = new
			// LinkedBlockingQueue<String>();
			// final LinkedBlockingQueue<String> waitLock2 = new
			// LinkedBlockingQueue<String>();
			new Thread(new Runnable() {
				@Override
				public void run() {
					try {
						BufferedReader br = new BufferedReader(
								new InputStreamReader(is));

						String line;
						while ((line = br.readLine()) != null)
							logQueue.add(line);
						//							waitLock2.add("");
						//							if (line.substring(line.length() - 2).equals("$ ")) {
						//								waitLock2.clear();
						//								waitLock1.add("$");
						//							}
					} catch (Exception e) {
						logQueue.add(e.getMessage());
					}
				}
			}).start();

			// waitLock1.take(); // $ sign was sent

			PrintWriter out = new PrintWriter(channel.getOutputStream());
			out.println("roslaunch turtlebot_teleop keyboard_teleop.launch");
			out.flush();

			// waitLock2.take(); // messages after sent command are received
			Thread.sleep(1000);
			logQueue();

			// channel.disconnect();
			// session.disconnect();
			// log("Disconnected.");
			// setDrivingEnabled(false);
			// setInputEnabled(true);

		} catch (JSchException e) {
			log(e.getMessage());
			setDrivingEnabled(false);
			setInputEnabled(true);
		} catch (Exception e) {
			log(e.getMessage());
			setDrivingEnabled(false);
			setInputEnabled(true);
		}
	}

	@Override
	public void onBackPressed() {
		super.onBackPressed();

		if ((session != null) && session.isConnected())
			session.disconnect();

		log("Finish");

		finish();
	}

	@Override
	protected void onPause() {
		super.onPause();

		if (driving)
			mSensorManager.unregisterListener(this);
	}

	@Override
	protected void onResume() {
		super.onResume();

		if (driving)
			mSensorManager.registerListener(this, mAccelerometer,
					SensorManager.SENSOR_DELAY_UI);

		log("Resume");
	}

	private void driveState(MotionEvent event) {

		boolean newDrivingState = (event.getAction() == MotionEvent.ACTION_DOWN)
		|| (event.getAction() == MotionEvent.ACTION_MOVE);

		if (driving == newDrivingState)
			return;

		driving = newDrivingState;

		if (driving)
			mSensorManager.registerListener(this, mAccelerometer,
					SensorManager.SENSOR_DELAY_NORMAL);
		else
			mSensorManager.unregisterListener(this);

		log(driving ? "Driving started." : "Driving stopped.");
		status("");

	}

	private void doInputStateChange() {
		boolean doEnable = host.getText().toString().contains("@");

		connectButton.setEnabled(doEnable);
		password.setEnabled(doEnable);

		log(doEnable ? "Input valid." : "Input non valid.");
	}

	private void setInputEnabled(boolean enabled) {
		host.setEnabled(enabled);
		password.setEnabled(enabled);
		connectButton.setEnabled(enabled);

		log(enabled ? "Input enabled." : "Input disabled.");
	}

	private void setDrivingEnabled(boolean enabled) {
		driveButton.setEnabled(enabled);

		log(enabled ? "Driving enabled." : "Driving disabled.");
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
			return;

		float floatX = 0;
		float floatY = 0;

		switch (mDisplay.getRotation()) {
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

		float tmpAccuracy;
		try {
			tmpAccuracy = Float.parseFloat(accuracy.getText().toString());
		} catch (NumberFormatException e) {
			tmpAccuracy = INITIAL_ACCURACY;
		}

		int intX = Math.round(floatX * tmpAccuracy);
		int intY = Math.round(floatY * tmpAccuracy);

		status("Driving... (" + intX + ", " + intY + ")");

		if ((intX == 0) && (intY == 0))
			return;

		try {

			PrintWriter out = new PrintWriter(channel.getOutputStream());

			while ((intX != 0) && (intY != 0)) {
				if (intX > 0) {
					intX--;
					for (int i = 0; i < new Integer(speed.getText().toString()); ++i) {
						out.print(KEYCODE_L);
						out.flush();
					}
				}
				if (intX < 0) {
					intX++;
					for (int i = 0; i < new Integer(speed.getText().toString()); ++i) {
						out.print(KEYCODE_R);
						out.flush();
					}
				}
				if (intY > 0) {
					intY--;
					for (int i = 0; i < new Integer(speed.getText().toString()); ++i) {
						out.print(KEYCODE_D);
						out.flush();
					}
				}
				if (intY < 0) {
					intY++;
					for (int i = 0; i < new Integer(speed.getText().toString()); ++i) {
						out.print(KEYCODE_U);
						out.flush();
					}
				}
			}

		} catch (Exception e) {
			log(e.getMessage());
		}

	}

	private void status(String s) {
		status.setText(s);
		logQueue();
	}

	private void logQueue() {

		while (!logQueue.isEmpty()) {
			Log.i(LOG_TAG, logQueue.peek());
			log.append(logQueue.poll() + "\n");
			logScroll.fling(500);
		}

	}

	private void log(String logMsg) {

		logQueue.add(logMsg);

		logQueue();

	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}
}