package common.parameter;

import java.util.Iterator;

public class IntParameterRange extends NumberParameterRange<Integer> {

	private class IntIter implements Iterator<Parameter<?>> {
		@Override
		public boolean hasNext() {
			return next <= max;
		}

		@Override
		public IntParameter next() {
			IntParameter p = new IntParameter(name, next);

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

	public IntParameterRange(String name, int min, int max, int step) {
		super (name, min, max, step == 0 ? Math.max(1, max - min) : step);
		
		if (min > max) {
			throw new IllegalArgumentException("min (" + min
					+ ") is smaller than max (" + max + ")");
		}
	}

	public IntParameterRange(String name, int i) {
		this(name, i, i, 0);
	}

	@Override
	public Iterator<Parameter<?>> iterator() {
		resetIterator();
		return new IntIter();
	}
}