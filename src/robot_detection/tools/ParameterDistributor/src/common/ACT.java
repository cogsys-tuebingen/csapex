package common;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;

public class ACT {
	private static long next = 1;
	
	private long id;

	private ACT(long id) {
		this.id = id;
	}

	public static synchronized ACT create() {
		return new ACT(next++);
	}
	
	@Override
	public String toString() {
		return Long.toString(id);
	}
	
	@Override
	public boolean equals(Object obj) {
		if(!(obj instanceof ACT)){
			return false;
		}
		
		return ((ACT) obj).id == id;
	}
	
	@Override
	public int hashCode() {
		return ((Long) id).hashCode();
	}

	public static ACT readFrom(BufferedReader reader) throws IOException {
		String line = reader.readLine();
		Long i = Long.valueOf(line);
		if(i == 0){
			throw new IllegalArgumentException("illegal ACT=" + line);
		}
		return new ACT(i);
	}

	public void writeTo(BufferedWriter writer) throws IOException {
		writer.write(this.toString() + '\n');
	}
}
