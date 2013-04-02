package common.parameter;

public class DoubleParameter extends Parameter<Double> {
	private static final long serialVersionUID = Parameter.serialVersionUID;
	
	private double value;

	public DoubleParameter(String name, double value) {
		super(name);
		this.value = value;
	}

	public Double getValue() {
		return value;
	}
	@Override
	
	public String toString() {
		return super.toString() + String.valueOf(value);
	}
}
