package handlers;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;

import common.Result;
import common.Task;

public abstract class ConnectionHandler extends Thread {
	public interface ClientCallback {
		Result handle(Task task, BufferedWriter writer);
	}
	public interface ServerCallback {
		void handle(Task task, Result response);
	}

	protected Socket socket;
	protected BufferedReader reader;
	protected BufferedWriter writer;
	
	private boolean running = true;

	public ConnectionHandler() {
	}

	protected void spin() throws IOException {
		reader = new BufferedReader(new InputStreamReader(
				socket.getInputStream()));
		writer = new BufferedWriter(new OutputStreamWriter(
				socket.getOutputStream()));

		while (running) {
			running = tick();
		}
	}

	protected abstract boolean tick() throws IOException;

}