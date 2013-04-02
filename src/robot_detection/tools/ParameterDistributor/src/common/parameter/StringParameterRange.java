package common.parameter;

import java.util.Iterator;
import java.util.List;

public class StringParameterRange extends ParameterRange {

	private class StringIter implements Iterator<Parameter<?>> {
		@Override
		public boolean hasNext() {
			return next < values.size();
		}

		@Override
		public StringParameter next() {
			StringParameter p = new StringParameter(name, values.get(next));
			next++;

			return p;
		}

		@Override
		public void remove() {
			throw new UnsupportedOperationException();
		}
	}

	int next;
	List<String> values;

	public StringParameterRange(String name, List<String> values) {
		super(name);
		this.values = values;
		resetIterator();
	}

	@Override
	public Iterator<Parameter<?>> iterator() {
		resetIterator();
		return new StringIter();
	}

	private void resetIterator() {
		next = 0;
	}
}