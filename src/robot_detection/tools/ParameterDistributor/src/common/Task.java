package common;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import serial.Serializer;

import common.parameter.Parameter;

public class Task {
	private List<Parameter<?>> parameters;
	
	public Task(List<Parameter<?>> parameters) {
		this.parameters = parameters;
	}
	
	public Task() {
		parameters = new ArrayList<Parameter<?>>();
	}

	public Task fork() {
		return new Task(new ArrayList<Parameter<?>>(parameters));
	}

	public void add(Parameter<?> value) {
		parameters.add(value);
	}
	
	@Override
	public String toString() {
		return parameters.toString();
	}

	@SuppressWarnings("unchecked")
	public static Task readFrom(BufferedReader reader) throws IOException {
		Object o = Serializer.fromString(reader.readLine());
		if(!(o instanceof List)){
			throw new IllegalArgumentException();
		}
		return new Task((List<Parameter<?>>) o);
	}

	public void writeTo(BufferedWriter writer) throws IOException {
		writer.write(Serializer.toString(parameters) + '\n');
	}

	public String toYAML() {
		String YAML = "{";
		int i = 0;
		for(Parameter<?> p: parameters){
			if(i++ != 0){
				YAML += ", ";
			}
			YAML += p.toYAML();
		}
		
		return YAML + "}";
	}
}
