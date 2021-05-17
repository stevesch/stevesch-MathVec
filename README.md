# stevesch-MathVec

Provides classes for

  vector2 : 2-element vector

  vector3 : 3-element vector

  vector4 : 4-element vector

  Matrix4 : 4x4 matrix

  Quat: quaternion


Typical operations for 3D rendering are provided for each of the above classes (transformation, projection, etc.).

Use in combination with stevesch-Mesh and stevesch-Display for rendering 3D objects on an SPI TFT screen.
# Building and Running

Add library dependency, e.g.

```
lib_deps = 
  https://github.com/stevesch/stevesch-MathVec.git
```

<br/>
Include in your code:

```
#include <stevesch-MathVec.h>
```

Example code:
```
using stevesch::vector3;
using stevesch::vector4;
using stevesch::matrix4;
using stevesch::quat;
```

<br/>
Vector addition:

```
vector3 a(2.0f, 3.0f, 5.0f);
vector3 b(7.0f, 11.0f, 13.0f);
vector3 c;
c = a + b; // c == <9, 14, 18>; equivalent: vector3::add(c, a, b);
```

<br/>
Vector transformation by a matrix:

```
vector3 a, b;
matrix4 m;
b = a.transform(m); // b = m*a
```

<br/>
Vector transformation by a quaternion:

```
quat q;
vector3 axis(0.0f, 0.0f, 1.0f); // z axis (w ignored)
// already normalized in our case, but if not: axis.normalize();

// <axis> must be normalized for 'fromAxisAngle':
q.fromAxisAngle(axis, stevesch::degToRad(30.0f));

vector4 a(1.0f, 0.0f, 0.0f, 1.0f);
vector4 b;
q.rotate(b, a); // equivalent to b = m*a if q were converted to a matrix
```
<br/>
Notes:

- Some vector operations are only implemented for vector4 (not vector3).  When in doubt about the w-component of a vector4, set it to 1.0f.

- quaternions should almost always be normalized when using them as rotations.  If you create them properly (e.g. using fromAxisAngle or fromMatrix), this will usually not be a big issue, but if you see that a quaternion is not normalized, something has probably gone wrong.
