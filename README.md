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

. . .

vector3 a(2.0f, 3.0f, 5.0f)
vector3 b(7.0f, 11.0f, 13.0f);



```
