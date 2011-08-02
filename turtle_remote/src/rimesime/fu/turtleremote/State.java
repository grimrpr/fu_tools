package rimesime.fu.turtleremote;

import android.content.SharedPreferences;
import android.os.Handler;

public class State {

	// savable states
	static String varHost = "";
	static String varPassword = "";
	static float varAccuracyMultiplyer = 1.0f;
	static int varCommandMultiplyer = 1;
	static boolean varAutoconnect = false;

	// temp states/vars
	final static Drive drive = new Drive();
	final static Connection connection = new Connection();
	private static Handler handler;

	static void init(Handler handler2) {

		handler = handler2;

	}

	static void load(SharedPreferences prefs) {

		Log.l("Loading variables...");

		varHost = prefs.getString("host", varHost);
		varPassword = prefs.getString("password", varPassword);
		varAccuracyMultiplyer = prefs.getFloat("accuracymultiplyer",
				varAccuracyMultiplyer);
		varCommandMultiplyer = prefs.getInt("commandmultiplyer",
				varCommandMultiplyer);
		varAutoconnect = prefs.getBoolean("autoconnect", varAutoconnect);

		Log.l("Loading finished.");

	}

	static void persist(SharedPreferences prefs) {

		Log.l("Saving variables...");

		SharedPreferences.Editor editor = prefs.edit();

		editor.putString("host", varHost);
		editor.putString("password", varPassword);
		editor.putFloat("accuracymultiplyer", varAccuracyMultiplyer);
		editor.putInt("commandmultiplyer", varCommandMultiplyer);
		editor.putBoolean("autoconnect", varAutoconnect);

		editor.commit();

		Log.l("Saving finished.");

	}

	static void operateOnViewThread(Runnable runnable) {

		if(handler == null) return;
		
		synchronized (handler) {
			handler.post(runnable);
		}
		
	}

}
