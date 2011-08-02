package rimesime.fu.turtleremote;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.Properties;

import com.jcraft.jsch.Channel;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class Connection {

	boolean loggedIn = false;

	private static Properties config = new Properties();
	static {
		config.put("StrictHostKeyChecking", "no");
	}

	private JSch ssh = new JSch();
	private Session session;
	private Channel channel;
	private PrintWriter printer;

	void connect(String host, String password) {
		// setInputEnabled(false);
		Log.l("Connecting...");

		ssh = new JSch();
		String user = host.substring(0, host.indexOf('@'));
		host = host.substring(host.indexOf('@') + 1);

		try {
			session = ssh.getSession(user, host, 22);
			session.setConfig(config);
			session.setPassword(password);
			session.connect();
			channel = session.openChannel("shell");
			channel.connect(5000);
			printer = new PrintWriter(channel.getOutputStream());

			Log.l("Connected! Start driving!");
			loggedIn = true;
			// setDrivingEnabled(true);

			final InputStream is = channel.getInputStream();
			new Thread(new Runnable() {
				public void run() {
					try {
						BufferedReader br = new BufferedReader(
								new InputStreamReader(is));

						String line;
						while ((line = br.readLine()) != null)
							Log.l(line);
					} catch (Exception e) {
						Log.l(e.getMessage());
					}
				}
			}).start();

			println("roslaunch turtlebot_teleop keyboard_teleop.launch");

			// Switch to main view
			State.operateOnViewThread(new Runnable() {
				public void run() {
					Turtle_RemoteActivity.trObj
							.switchContentView(R.layout.main);
				}
			});

			// Store connection infos for next program startup
			State.varHost = user + "@" + host;
			State.varPassword = password;
			
		} catch (JSchException e) {
			Log.l(e.getMessage());
			// setDrivingEnabled(false);
			// setInputEnabled(true);
		} catch (Exception e) {
			Log.l(e.getMessage());
			// setDrivingEnabled(false);
			// setInputEnabled(true);
		}
	}

	void disconnect() {

		loggedIn = false;
		channel.disconnect();
		session.disconnect();
		Log.l("Disconnected.");
		// setDrivingEnabled(false);
		// setInputEnabled(true);

	}

	void print(char c) {
		try {
			printer.print(c);
			printer.flush();
		} catch (Exception e) {
			Log.l(e.getMessage());
		}
	}

	void println(String s) {
		try {
			printer.println(s);
			printer.flush();
		} catch (Exception e) {
			Log.l(e.getMessage());
		}
	}

}
