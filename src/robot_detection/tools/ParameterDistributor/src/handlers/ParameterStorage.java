package handlers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import common.Result;
import common.Task;
import common.parameter.Parameter;
import common.parameter.ParameterRange;

public class ParameterStorage {

	private List<ParameterRange> parameters;

	private Map<Task, Result> combinations;

	private List<Task> open;
	private List<Task> inProgress;

	public ParameterStorage() {
		parameters = new ArrayList<ParameterRange>();
		open = new ArrayList<Task>();
		inProgress = new ArrayList<Task>();
	}
	
	public void add(ParameterRange p) {
		parameters.add(p);
	}

	public void create() {
		combinations = new HashMap<Task, Result>();
		Task line = new Task();
		createLevel(0, line);

		for (Task key : combinations.keySet()) {
			System.out.println(key.toString() + "\t->\t"
					+ combinations.get(key));
		}
	}

	public void createLevel(int level, Task line) {
		for (Parameter<?> value : parameters.get(level)) {
			Task fork = line.fork();
			fork.add(value);

			if (level >= parameters.size() - 1) {
				open.add(fork);
				System.out.println(parameters.get(level).getName() + " - " + fork);
			} else {
				createLevel(level + 1, fork);
			}
		}
	}

	public Task getNewTask() {
		Task packet = null;
		
		if (open.size() != 0) {
			packet = open.get(0);
			open.remove(0);
			inProgress.add(packet);
			
		} else if(inProgress.size() != 0) {
			while(inProgress.size() > 0){
				Task e = inProgress.get(0);
				inProgress.remove(0);
				if(!combinations.containsKey(e)){
					packet = e;
					inProgress.add(e);
					break;
				}
			}
		}
		
		return packet;
	}

	public void setResultFor(Task candidate, Result answer) {
		inProgress.remove(candidate);
		combinations.put(candidate, answer);		
	}

	public int getTaskCount() {
		return open.size();
	}
}
