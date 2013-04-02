package common.parameter;


public abstract class NumberParameterRange <T extends Number> extends ParameterRange {
	protected T min;
	protected T max;
	protected T step;

	protected T next;

	public NumberParameterRange(String name, T min, T max, T step) {
		super(name);
				
		this.min = min;
		this.max = max;
		this.step = step;

		resetIterator();
	}

	protected void resetIterator() {
		next = min;
	}

	public T getMin() {
		return min;
	}

	public T getMax() {
		return max;
	}

	public T getStep() {
		return step;
	}
}