package handlers;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;

import common.ACT;
import common.Result;
import common.Task;

public class ParameterServerThread extends ConnectionHandler {
	private ParameterStorage storage;

	private Map<ACT, Task> inProgress;
	private ServerCallback callback;

	public ParameterServerThread(Socket socket, ParameterStorage storage,
			ServerCallback callback) {
		this.socket = socket;
		this.storage = storage;
		this.inProgress = new HashMap<ACT, Task>();
		this.callback = callback;
	}

	public void run() {
		try {
			spin();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	protected void spin() throws IOException {
		System.out.println("client connected");
		super.spin();
		System.out.println("client disconnected");
	}
	
	protected synchronized boolean tick() throws IOException {
		String method = reader.readLine();

		if(method == null || method.equals("bye")){
			return false;
			
		} else if (method.equals("get")) {
			sendNewTask(writer);
		} else {
			readAnswer(reader);
		}
		
		return true;
	}

	public void sendNewTask(BufferedWriter writer) throws IOException {
		Task packet = storage.getNewTask();

		if (packet != null) {
			System.out.println("sending new task to client:");
			System.out.println(packet);
			System.out.println(storage.getTaskCount());
			
			writer.write("task\n");
			ACT act = ACT.create();
			inProgress.put(act, packet);

			act.writeTo(writer);
			packet.writeTo(writer);

		} else {
			writer.write("done, thank you for your cooperation\n");
		}
		writer.flush();
	}

	public void readAnswer(BufferedReader reader) throws IOException {
		ACT act = ACT.readFrom(reader);

		Result answer = Result.readFrom(reader);
		Task task = inProgress.get(act);
		inProgress.remove(act);

		if (task != null) {
			storage.setResultFor(task, answer);
			callback.handle(task, answer);
		} else {
			System.err.println("illegal answer");
		}
	}
}
