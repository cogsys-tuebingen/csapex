package common.parameter;

public class StringParameter extends Parameter<String> {
	private static final long serialVersionUID = Parameter.serialVersionUID;
	
	private String value;

	public StringParameter(String name, String value) {
		super(name);
		this.value = value;
	}

	public String getValue() {
		return value;
	}
	@Override
	public String toString() {
		return super.toString() + value;
	}
}
