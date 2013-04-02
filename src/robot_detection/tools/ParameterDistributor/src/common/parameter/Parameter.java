package common.parameter;

import java.io.Serializable;

public abstract class Parameter<T> implements Serializable {
	static final long serialVersionUID = 1L;
	
	private String name;
	
	public Parameter(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}
	
	public abstract T getValue();
	
	@Override
	public String toString() {
		return name + "=";
	}

	public String toYAML() {
		return name + ": " + getValue();
	}
}
