package exe;

import handlers.ParameterClient;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;

import common.Result;
import common.Task;

public class Client implements ParameterClient.ClientCallback {
	public static void main(String[] args) {
		Client c = new Client();
		try {
			c.run();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void run() throws IOException {
		ParameterClient client;
		client = new ParameterClient(this);

		client.connectTo("localhost", 1337);
	}

	public Result handle(Task task, BufferedWriter writer) {
		System.out.println("task: " + task);

		String cmd = System.getenv().get("RABOT")
				+ "/ros/vision/robot_detection/bin/trainer_batch";
		cmd += " " + task.toYAML();

		try {
			System.out.println(cmd);
			Process pr = Runtime.getRuntime().exec(cmd);
			BufferedReader input = new BufferedReader(new InputStreamReader(
					pr.getInputStream()));
			BufferedReader error = new BufferedReader(new InputStreamReader(
					pr.getErrorStream()));

			String line = null;
			while ((line = input.readLine()) != null) {
				System.out.println(line);
			}
			while ((line = error.readLine()) != null) {
				System.err.println(line);
				throw new IllegalAccessException();
			}

			int exitVal = pr.waitFor();
			System.out.println(exitVal);
			Thread.sleep(1000);
			return new Result("ok");

		} catch (Exception e) {
			e.printStackTrace();
			return new Result("error");
		}

	}
}
