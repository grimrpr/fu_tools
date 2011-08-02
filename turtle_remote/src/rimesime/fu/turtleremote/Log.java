package rimesime.fu.turtleremote;

import android.widget.ScrollView;
import android.widget.TextView;

public class Log {

	static final String LOG_TAG = "TurtleRemote";

	static void l(final String s) {

		State.operateOnViewThread(new Runnable() {
			public void run() {

//				Toast.makeText(Turtle_RemoteActivity.trObj, s,
//						Toast.LENGTH_SHORT).show();

				android.util.Log.i(LOG_TAG, s);

				TextView lblLog = (TextView) Turtle_RemoteActivity.trObj
						.findViewById(R.id.log);
				if (lblLog == null)
					return;
				lblLog.append(s + "\n");
				ScrollView logScroll = (ScrollView) Turtle_RemoteActivity.trObj
						.findViewById(R.id.logScroll);
				if (logScroll == null)
					return;
				logScroll.fling(500);

			}
		});

	}

	static void s(final String s) {

		State.operateOnViewThread(new Runnable() {
			public void run() {

				TextView lblStatus = (TextView) Turtle_RemoteActivity.trObj
						.findViewById(R.id.status);
				if (lblStatus == null)
					return;
				lblStatus.setText(s);

			}
		});

	}

}
