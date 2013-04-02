package common.parameter;

public class IntParameter extends Parameter<Integer> {
	private static final long serialVersionUID = Parameter.serialVersionUID;
	
	private int value;

	public IntParameter(String name, int value) {
		super(name);
		this.value = value;
	}

	public Integer getValue() {
		return value;
	}
	
	@Override
	public String toString() {
		return super.toString() + String.valueOf(value);
	}
}
