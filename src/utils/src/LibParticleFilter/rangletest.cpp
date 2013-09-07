
#include "rangle.h"
#include <iostream>

using namespace std;

int main() {

Angle a(0);
Angle b(0);
Angle c(0);

a.setDegrees(10);
b.setDegrees(-10);
c.setDegrees(191);

a.normalize();
b.normalize();
c.normalize();

a += b;

cout << a.getDegrees() << endl;
cout << b.getDegrees() << endl;
cout << c.getDegrees() << endl;

if (b.isInRange(a,c)) {
	cout << "abc" << endl;
}
else {
	if (b.shortest_angular_distance(a) < b.shortest_angular_distance(c))
		cout << "->a" << endl;
	else
		cout << "->c" << endl;
}


return 0;
}
