package common.parameter;

import java.util.Iterator;

public class DoubleParameterRange extends NumberParameterRange<Double> {

	private class DoubleIter implements Iterator<Parameter<?>> {
		@Override
		public boolean hasNext() {
			return next <= max;
		}

		@Override
		public DoubleParameter next() {
			DoubleParameter p = new DoubleParameter(name, next);

			if(next < max && next + step > max){
				next = max;
			} else {
				next += step;
			}


			return p;
		}

		@Override
		public void remove() {
			throw new UnsupportedOperationException();
		}
	}

	public DoubleParameterRange(String name, double min, double max, double step) {
		super (name, min, max, Math.abs(step) < 0.001 ? Math.signum(step) * 0.001 : step);
		
		if (min > max) {
			throw new IllegalArgumentException("min (" + min
					+ ") is smaller than max (" + max + ")");
		}
	}

	@Override
	public Iterator<Parameter<?>> iterator() {
		resetIterator();
		return new DoubleIter();
	}
}