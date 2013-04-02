package handlers;

import handlers.ConnectionHandler.ServerCallback;

import java.io.IOException;
import java.net.ServerSocket;

import common.parameter.ParameterRange;

public class ParameterServer {
	private ParameterStorage storage;

	private ServerSocket socket;
	private boolean running = true;

	private ServerCallback callback;

	public ParameterServer(int port, ServerCallback callback)
			throws IOException {
		storage = new ParameterStorage();
		socket = new ServerSocket(port);
		this.callback = callback;
	}

	public void create() {
		storage.create();
	}

	public void add(ParameterRange p) {
		storage.add(p);
	}

	public void spin() throws IOException {
		System.out.println("spinning with " + storage.getTaskCount() + " tasks");
		while (running) {
			new ParameterServerThread(socket.accept(), storage, callback)
					.start();
		}
	}
}
