package handlers;

import java.io.IOException;
import java.net.Socket;

import common.ACT;
import common.Result;
import common.Task;

public class ParameterClient extends ConnectionHandler {

	private ClientCallback handler;
	
	public ParameterClient(ClientCallback handler) throws IOException {
		this.handler = handler;
	}

	public void connectTo(String address, int port) throws IOException {
		socket = new Socket(address, port);

		spin();
		
		writer.write("bye\n");
		writer.flush();
		
		socket.close();
	}

	protected boolean tick() throws IOException {
		method("get");
		writer.flush();

		String response = reader.readLine();

		if (response.substring(0, 4).equals("done")) {
			System.out.println(response);
			return false;

		} else if(response.substring(0, 4).equals("task")){
			ACT act = ACT.readFrom(reader);
			Task task = Task.readFrom(reader);
			
			Result answer = handler.handle(task, writer);
			System.out.println("answer=" + answer);

			method("post");
			act.writeTo(writer);
			answer.writeTo(writer);
			writer.flush();

			return true;
			
		} else {
			throw new IllegalStateException("cannot handle: " + response);
		}
	}

	private void method(String method) throws IOException {
		writer.write(method + "\n");
	}
}
