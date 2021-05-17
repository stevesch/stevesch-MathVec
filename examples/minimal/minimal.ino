#include <Arduino.h>
// This is a test example for MathVec
#include <stevesch-MathVec.h>

using stevesch::vector2;
using stevesch::vector3;
using stevesch::vector4;
using stevesch::matrix4;
using stevesch::quat;


Print& operator<<(Print& o, const char* str) { o.print(str); return o; }
Print& operator<<(Print& o, const char ch) { o.print(ch); return o; }

Print& operator<<(Print& o, const vector2& v) { o.printf("<%5.2f, %5.2f>", v.x, v.y); return o; }
Print& operator<<(Print& o, const vector3& v) { o.printf("<%5.2f, %5.2f, %5.2f>", v.x, v.y, v.z); return o; }
Print& operator<<(Print& o, const vector4& v) { o.printf("<%5.2f, %5.2f, %5.2f, %5.2f>", v.x, v.y, v.z, v.w); return o; }
Print& operator<<(Print& o, const quat& q) { o.printf("<%5.2f, %5.2f, %5.2f, %5.2f>", q.X(), q.Y(), q.Z(), q.A()); return o; }

void printMxb(Print& o, const matrix4& m, const vector4& x, const vector4& b) {
  vector4 r;
  for (int i=0; i<4; ++i) {
    r = m.getRow(i);
    o.printf("[%5.2f, %5.2f, %5.2f, %5.2f][ %5.2f ] ", r.x, r.y, r.z, r.w, x[i]);
    char ch = (i == 2) ? '=' : ' ';
    o << ch << ' ';
    o.printf("[ %5.2f ]\n", b[i]);
  }
}

// print q*x*q' = b
void printqxqb(Print& o, const quat& q, const vector3& x, const vector3& b)
{
  quat qc(q);
  qc.conj();
  o.printf("[ %5.2f ]  [ %5.2f ]                                        [ %5.2f ]\n",
    q.X(), x.x, b.x);
  o.printf("[ %5.2f ]  [ %5.2f ]                                        [ %5.2f ]\n",
    q.Y(), x.y, b.y);
  o.printf("[ %5.2f ]  [ %5.2f ] [ %5.2f ][ %5.2f ][ %5.2f ][ %5.2f ] = [ %5.2f ]\n",
    q.Z(), x.z, qc.X(), qc.Y(), qc.Z(), qc.A(), b.z);
  o.printf("[ %5.2f ]  [ %5.2f ]                                        [ %5.2f ]\n",
    q.A(), 1.0f, 1.0f);
}


void printAddition()
{
  vector3 a(2.0f, 3.0f, 5.0f);
  vector3 b(7.0f, 11.0f, 13.0f);
  vector3 c;
  c = a + b; // or vector3::add(c, a, b);
  Serial << "Vector addition:\n";
  Serial << "  a: " << a << '\n';
  Serial << "+ b: " << b << '\n';
  Serial << "= c: " << c << '\n';
}


void printRotationMatrix()
{
  matrix4 mtxR;
  mtxR.zMatrix(stevesch::degToRad(30.0f));
  vector4 a(1.0f, 0.0f, 0.0f, 1.0f);
  vector4 b(a);
  b.transform(mtxR);  // b = R*a
  Serial << "Rotation matrix, 30 degrees around z axis:\n";
  printMxb(Serial, mtxR, a, b);
}

void printRotationQuat()
{
  // same 30-degree rotation about Z, but using quat and arbitrary axis-angle conversion:
  Serial << "Rotation quat, 30 degrees around z axis:\n";
  quat q;
  vector3 axis(0.0f, 0.0f, 1.0f); // z axis (w ignored)
  // already normalized in our case, but if not: axis.normalize();
  q.fromAxisAngle(axis, stevesch::degToRad(30.0f)); // note <axis> must be normalized

  vector3 a(1.0f, 0.0f, 0.0f);
  vector3 b;
  q.rotate(b, a);
  printqxqb(Serial, q, a, b);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Setup initializing...");

  printAddition();
  Serial.println();
  printRotationMatrix();
  Serial.println();
  printRotationQuat();

  Serial.println("Setup complete.");
}

void loop()
{
  const long kLogInterval = 5000;
  static long nextLog = kLogInterval;
  long now = millis();
  if (now >= nextLog) {
    nextLog += kLogInterval;
    Serial.println("Reset to re-run tests/demo");
  }
}
