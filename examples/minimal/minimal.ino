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

void printMxb(Print& o, const matrix4& m, const vector3& x, const vector3& b) {
  vector4 r;
  for (int i=0; i<4; ++i) {
    r = m.getRow(i);
    float xi = (i < 3) ? x[i] : 1.0f;
    float bi = (i < 3) ? b[i] : 1.0f;
    o.printf("[%5.2f, %5.2f, %5.2f, %5.2f][ %5.2f ] ", r.x, r.y, r.z, r.w, xi);
    char ch = (i == 2) ? '=' : ' ';
    o << ch << ' ';
    o.printf("[ %5.2f ]\n", bi);
  }
}

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


void printAdd3()
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

void printDot3()
{
  vector3 a(2.0f, 3.0f, 5.0f);
  vector3 b(7.0f, 11.0f, 13.0f);
  float c = a.dot(b);
  Serial << "Vector dot:\n";
  Serial << "  a: " << a << '\n';
  Serial << ". b: " << b << '\n';
  Serial << "= c: ";
  Serial.print(c, 2);
  Serial << " (expected 112.0)\n";
}

void printCross3()
{
  vector3 a(2.0f, 3.0f, 5.0f);
  vector3 b(7.0f, 11.0f, 13.0f);
  vector3 c;
  vector3::cross(c, a, b);
  Serial << "Vector cross:\n";
  Serial << "  a: " << a << '\n';
  Serial << "x b: " << b << '\n';
  Serial << "= c: " << c << '\n';
  Serial << "a . c = ";
  Serial.print(a.dot(c), 2);
  Serial << " (expected 0)\n";
  Serial << "b . c = ";
  Serial.print(b.dot(c), 2);
  Serial << " (expected 0)\n";
}

void printRotationMatrix3()
{
  matrix4 mtxR;
  mtxR.zMatrix(stevesch::degToRad(30.0f));
  vector3 a(1.0f, 0.0f, 0.0f);
  vector3 b(a);
  b.transform(mtxR);    // b = R*<a, 1>
  vector3 c = mtxR * a; // c = R*<a, 1>

  Serial << "Rotation matrix, 30 degrees around z axis (using v3 .transform):\n";
  printMxb(Serial, mtxR, a, b);

  Serial << "Rotation matrix, 30 degrees around z axis (using v3 operator*):\n";
  printMxb(Serial, mtxR, a, c);
}

void printRotationMatrix4()
{
  matrix4 mtxR;
  mtxR.zMatrix(stevesch::degToRad(30.0f));
  vector4 a(1.0f, 0.0f, 0.0f, 1.0f);
  vector4 b(a);
  b.transform(mtxR);    // b = R*a
  vector4 c = mtxR * a; // c = R*a

  Serial << "Rotation matrix, 30 degrees around z axis (using v4 .transform):\n";
  printMxb(Serial, mtxR, a, b);

  Serial << "Rotation matrix, 30 degrees around z axis (using v4 operator*):\n";
  printMxb(Serial, mtxR, a, c);
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

  printAdd3();
  printDot3();
  printCross3();
  Serial.println();
  printRotationMatrix3();
  printRotationMatrix4();
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
