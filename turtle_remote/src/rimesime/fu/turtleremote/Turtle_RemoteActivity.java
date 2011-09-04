package rimesime.fu.turtleremote;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.Display;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnKeyListener;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.EditText;
import android.widget.ToggleButton;

public class Turtle_RemoteActivity extends Activity {

	// access to all (makes it easier)
	public static Turtle_RemoteActivity trObj;

	// main content sensors
	private static SensorManager sensorManager;
	private static Sensor accelerometer;
	private static WindowManager windowManager;
	static Display display;

	// Handler
	private Handler handler = new Handler();

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		trObj = this;
		State.init(handler);

		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

		State.load(getPreferences(MODE_PRIVATE));

		if (!State.connection.loggedIn)
			switchContentView(R.layout.login);
		else
			switchContentView(R.layout.main);

		if (!State.connection.loggedIn && State.varAutoconnect)
			State.connection.connect(State.varHost, State.varPassword);

		// sensors
		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		accelerometer = sensorManager
				.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		windowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
		display = windowManager.getDefaultDisplay();

		Log.l("Created");

	}

	void switchContentView(int id) {

		if (id == R.layout.login) {

			Log.l("Switching to login view.");

			this.setContentView(id);

			// login content view
			final EditText txtHost = (EditText) findViewById(R.id.host);
			final EditText txtPassword = (EditText) findViewById(R.id.password);
			final EditText txtRosMasterURI = (EditText) findViewById(R.id.rosmasteruri);
			final Button btnConnect = (Button) findViewById(R.id.connect);
			final ToggleButton tbtnAutoConnect = (ToggleButton) findViewById(R.id.autoConnect);

			tbtnAutoConnect.setChecked(State.varAutoconnect);

			txtHost.addTextChangedListener(new TextWatcher() {
				public void onTextChanged(CharSequence s, int start,
						int before, int count) {
					boolean doEnable = s.toString().contains("@");

					btnConnect.setEnabled(doEnable);
					txtPassword.setEnabled(doEnable);

					Log.l(doEnable ? "Input valid." : "Input non valid.");
				}

				public void beforeTextChanged(CharSequence s, int start,
						int count, int after) {
				}

				public void afterTextChanged(Editable s) {
				}
			});

			txtHost.setText(State.varHost);
			txtPassword.setText(State.varPassword);
			txtRosMasterURI.setText(State.varRosMasterURI);

			txtRosMasterURI.addTextChangedListener(new TextWatcher() {

				public void onTextChanged(CharSequence s, int start,
						int before, int count) {
					State.varRosMasterURI = s.toString();
				}

				public void beforeTextChanged(CharSequence s, int start,
						int count, int after) {
				}

				public void afterTextChanged(Editable s) {
				}
			});

			tbtnAutoConnect
					.setOnCheckedChangeListener(new OnCheckedChangeListener() {
						public void onCheckedChanged(CompoundButton buttonView,
								boolean isChecked) {
							State.varAutoconnect = isChecked;
						}
					});

			btnConnect.setOnClickListener(new OnClickListener() {
				public void onClick(View v) {
					State.connection.connect(txtHost.getText().toString(),
							txtPassword.getText().toString());
				}
			});
		} else if (id == R.layout.main) {

			Log.l("Switching to driving view.");

			this.setContentView(id);

			// main content view
			// final ScrollView scrollLog = (ScrollView)
			// findViewById(R.id.logScroll);
			// final TextView lblLog = (TextView) findViewById(R.id.log);
			// final TextView lblStatus = (TextView) findViewById(R.id.status);
			final EditText txtAccuracyMultiplyer = (EditText) findViewById(R.id.accuracy);
			final EditText txtCommandmultiplyer = (EditText) findViewById(R.id.speed);
			final Button btnDrive = (Button) findViewById(R.id.drive);

			txtCommandmultiplyer
					.setText(new Integer(State.varCommandMultiplyer).toString());
			txtAccuracyMultiplyer.setText(Float
					.toString(State.varAccuracyMultiplyer));

			txtCommandmultiplyer.setOnKeyListener(new OnKeyListener() {
				public boolean onKey(View v, int keyCode, KeyEvent event) {
					int i = -1;

					try {
						i = Integer.parseInt(txtCommandmultiplyer.getText()
								.toString());
						txtCommandmultiplyer.setBackgroundColor(Color.WHITE);
						State.varCommandMultiplyer = i;
					} catch (NumberFormatException e) {
						txtCommandmultiplyer.setBackgroundColor(Color.RED);
					}

					return false;
				}
			});

			txtAccuracyMultiplyer.setOnKeyListener(new OnKeyListener() {
				public boolean onKey(View v, int keyCode, KeyEvent event) {
					float i = -1f;

					try {
						i = Float.parseFloat(txtAccuracyMultiplyer.getText()
								.toString());
						txtAccuracyMultiplyer.setBackgroundColor(Color.WHITE);
						State.varAccuracyMultiplyer = i;
					} catch (NumberFormatException e) {
						txtAccuracyMultiplyer.setBackgroundColor(Color.RED);
					}

					return false;
				}
			});

			btnDrive.setOnTouchListener(new OnTouchListener() {
				public boolean onTouch(View v, MotionEvent event) {

					driveState(event);

					return false;

				}
			});

		}

	}

	@Override
	public void onBackPressed() {

		if (State.connection.loggedIn) {
			State.connection.disconnect();
			sensorManager.unregisterListener(State.drive);
			State.drive.isDriving = false;

			switchContentView(R.layout.login);
			return;
		} else {
			Log.l("Finish");
			super.onBackPressed();
			finish();
		}

	}

	@Override
	protected void onPause() {
		super.onPause();

		if (State.drive.isDriving) {
			sensorManager.unregisterListener(State.drive);
			State.drive.isDriving = false;
		}

		State.persist(getPreferences(MODE_PRIVATE));

		Log.l("Pause");
	}

	private void driveState(MotionEvent event) {

		if (!State.connection.loggedIn)
			return;

		boolean newIsDriving = (event.getAction() == MotionEvent.ACTION_DOWN)
				|| (event.getAction() == MotionEvent.ACTION_MOVE);

		if (State.drive.isDriving == newIsDriving)
			return;

		State.drive.isDriving = newIsDriving;

		if (State.drive.isDriving) {
			sensorManager.registerListener(State.drive, accelerometer,
					SensorManager.SENSOR_DELAY_NORMAL);
			State.drive.lockCurrentState();
		} else
			sensorManager.unregisterListener(State.drive);

		Log.l(State.drive.isDriving ? "Driving started." : "Driving stopped.");

		Log.s("");

	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.menu, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		ToggleButton tbtnAutoConnect;

		switch (item.getItemId()) {
		case R.id.ac_on:
			State.varAutoconnect = !item.isChecked();
			tbtnAutoConnect = (ToggleButton) findViewById(R.id.autoConnect);
			if (tbtnAutoConnect != null) {
				tbtnAutoConnect.setChecked(State.varAutoconnect);
			}
			return true;
		case R.id.ac_off:
			State.varAutoconnect = item.isChecked();
			tbtnAutoConnect = (ToggleButton) findViewById(R.id.autoConnect);
			if (tbtnAutoConnect != null) {
				tbtnAutoConnect.setChecked(State.varAutoconnect);
			}
			return true;
		default:
			return super.onOptionsItemSelected(item);
		}
	}

}