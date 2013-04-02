package common;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;

public class Result {
	public static final Result NO_RESULT = new Result("no result");
	
	private String description;

	public Result(String description) {
		this.description = description;
	}
	
	@Override
	public String toString() {
		return description;
	}

	public static Result readFrom(BufferedReader reader) throws IOException {
		return new Result(reader.readLine());
	}

	public void writeTo(BufferedWriter writer) throws IOException {
		writer.write(toString() + '\n');
	}
}
