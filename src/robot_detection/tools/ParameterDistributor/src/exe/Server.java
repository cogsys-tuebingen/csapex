package exe;

import handlers.ConnectionHandler.ServerCallback;
import handlers.ParameterServer;

import java.io.IOException;
import java.util.Arrays;

import common.Result;
import common.Task;
import common.parameter.DoubleParameterRange;
import common.parameter.IntParameterRange;
import common.parameter.StringParameterRange;

public class Server implements ServerCallback {
	public static void main(String[] args) {
		Server s = new Server();
		s.run();
	}

	public void run() {
		ParameterServer server;
		try {
			server = new ParameterServer(1337, this);

			// doc["descriptor_type"] >> descriptor;
			// doc["keypoint_type"] >> keypoint;
			// doc["extractor_threshold"] >> cfg.extractor_threshold;
			// doc["max_dist"] >> cfg.max_dist;
			// doc["min_points"] >> cfg.min_points;
			// doc["octaves"] >> cfg.octaves;
			// doc["preblur_size"] >> cfg.preblur_size;
			// doc["max_import_per_dir"] >> max_import_per_dir;

			server.add(new StringParameterRange("keypoint_type", Arrays.asList(
					"BRISK", "SURF", "SIFT", "ORB", "GFTT"))); // FAST = ORB
			// server.add(new StringParameterRange("descriptor_type",
			// Arrays.asList("BRISK")));
			server.add(new IntParameterRange("extractor_threshold", 20, 80, 10));
			server.add(new IntParameterRange("max_dist", 100));
			server.add(new IntParameterRange("min_points", 5));
			server.add(new IntParameterRange("octaves", 8));
			server.add(new IntParameterRange("max_import_per_dir", 250));
			server.add(new IntParameterRange("bin_count", 16));
			server.add(new IntParameterRange("use_pruning", 1));
			server.add(new IntParameterRange("crop_test", 0));
			server.add(new IntParameterRange("interactive", 0));
			// server.add(new IntParameterRange("preblur_size", 0, 3, 1));

			server.create();

			server.spin();

		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void handle(Task task, Result answer) {
		System.out.println(task + " => " + answer);
	}
}
