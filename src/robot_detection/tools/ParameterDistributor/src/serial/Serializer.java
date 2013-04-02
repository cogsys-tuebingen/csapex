package serial;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import biz.source_code.base64Coder.Base64Coder;

public class Serializer {

	public static String toString(Object obj) {
		try {
			ByteArrayOutputStream os = new ByteArrayOutputStream();
			ObjectOutputStream oos = new ObjectOutputStream(os);
			oos.writeObject(obj);
			oos.close();
			return new String(Base64Coder.encode(os.toByteArray()));

		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}

	public static Object fromString(String line) {
		try {
			byte[] data = Base64Coder.decode(line);
			ObjectInputStream ois = new ObjectInputStream(
					new ByteArrayInputStream(data));
			Object o = ois.readObject();
			ois.close();
			return o;

		} catch (IOException e) {
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		}
		return null;
	}

}
