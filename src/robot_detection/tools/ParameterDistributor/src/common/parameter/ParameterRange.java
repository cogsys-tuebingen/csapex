package common.parameter;

public abstract class ParameterRange implements Iterable<Parameter<?>> {
	protected String name;
	
	public ParameterRange(String name) {
		this.name = name;
	}
	
	public String getName() {
		return name;
	}
}
