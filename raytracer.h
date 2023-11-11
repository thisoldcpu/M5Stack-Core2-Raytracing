/*------------------------------------------------------------------------
  This is based on Paul Heckbert's "business card raytracer".
  His site is: http://www.cs.cmu.edu/~ph/

  I modified it a lot adding colors/materials, arbitrary sphere
  positions, etc.
  
  I also added some comments and made it readable. The original
  code was designed to be small enough to print on the back of
  a business card (hence the name) so it was very hard to read.
  
  FTB.
------------------------------------------------------------------------*/

/*------------------------------------------------------------------------
  Values you can play with...
------------------------------------------------------------------------*/

// Position of the camera (nb. 'Z' is up/down)
float cameraX = 0.0;
float cameraY = -3.0;
float cameraZ = 6.0;

// What the camera is pointing at
float targetX = 0.0;
float targetY = 0.0;
float targetZ = 4.0;

// Camera orbit parameters
float orbit_radius = 8.0;
float orbit_angle_degrees = 0.0;    // Starting angle in degrees
const float angle_increment = 0.5;  // The increment in degrees for each frame (adjust as needed to control speed)

// We cast this many rays per pixel for stochastic antialiasing and soft-shadows.
//
// Large numbers produce a nicer image but it runs a lot slower
//static const int raysPerPixel = 4;

const int MAX_DEPTH = 5;                   // Maximum recursion depth for reflections
const float FLOOR_REFLECTIVITY = 0.3f;     // Reflectivity of the floor, adjust as needed
static const float fov = 0.60f;            // The camera's field of view, smaller=>zoom, larger=>wide angle
static const float shadowRegion = 0.125f;  // The size of the soft shadow, larger=>wider area
static const float ambient = 0.05f;
static const float materials[] PROGMEM = {
  0.8f, 0.8f, 0.8f, 0.5f,  // Mirror
  1.0f, 0.0f, 0.0f, 0.3f,  // Red
  0.0f, 1.0f, 0.0f, 0.2f,  // Green
  0.0f, 0.0f, 1.0f, 0.3f,  // Blue
  0.0f, 0.8f, 0.8f, 0.2f,  // Cyan
  0.8f, 0.0f, 0.8f, 0.3f,  // Magenta
  1.0f, 0.5f, 0.0f, 0.2f,  // Orange
  1.0f, 1.0f, 0.0f, 0.3f,  // Yellow
  0.5f, 0.0f, 0.5f, 0.2f,  // Purple
  0.0f, 1.0f, 1.0f, 0.3f,  // Bright Cyan
  0.0f, 0.5f, 0.0f, 0.2f,  // Dark Green
  0.5f, 0.2f, 0.0f, 0.3f,  // Brown
  1.0f, 0.8f, 0.6f, 0.2f,  // Skin or Peach
  0.8f, 0.8f, 0.4f, 0.2f,  // Color 14
  0.3f, 0.5f, 0.4f, 0.2f,  // Color 15
  0.8f, 0.3f, 0.5f, 0.2f,  // Color 16
  0.6f, 0.9f, 0.5f, 0.2f,  // Color 17
  0.3f, 0.8f, 0.6f, 0.2f,  // Color 18
  0.3f, 0.9f, 1.0f, 0.2f,  // Color 19
  0.8f, 0.9f, 0.3f, 0.2f,  // Color 20
  0.7f, 0.9f, 0.7f, 0.2f,  // Color 21
  0.5f, 0.1f, 0.4f, 0.2f,  // Color 22
  0.6f, 0.9f, 1.0f, 0.2f,  // Color 23
  0.5f, 0.9f, 0.3f, 0.2f,  // Color 24
  0.8f, 0.5f, 0.0f, 0.2f,  // Color 25
  0.7f, 0.4f, 0.8f, 0.2f,  // Color 26
  0.7f, 0.0f, 0.5f, 0.2f,  // Color 27
  0.9f, 0.2f, 0.3f, 0.2f,  // Color 28
  0.9f, 0.2f, 0.6f, 0.2f,  // Color 29
  0.2f, 1.0f, 0.8f, 0.2f,  // Color 30
  0.4f, 0.1f, 0.3f, 0.2f,  // Color 31
  0.5f, 0.9f, 0.1f, 0.2f,  // Color 32
  0.6f, 0.7f, 0.5f, 0.2f,  // Color 33
  0.8f, 0.5f, 1.0f, 0.2f,  // Color 34
  0.6f, 0.6f, 0.4f, 0.2f,  // Color 35
  0.6f, 0.4f, 0.6f, 0.2f,  // Color 36
  0.3f, 0.2f, 0.2f, 0.2f,  // Color 37
  1.0f, 0.0f, 0.0f, 0.3f,  // Color 38
  0.8f, 0.0f, 0.8f, 0.3f,  // Color 39
  0.8f, 0.8f, 0.8f, 0.5f,  // Color 40
  0.0f, 1.0f, 0.0f, 0.2f,  // Color 41
  0.0f, 0.0f, 1.0f, 0.3f,  // Color 42
  0.0f, 0.8f, 0.8f, 0.2f   // Color 43
};

#define NUM_SPHERES 25
static const float spheres[] PROGMEM = {
  25.0f, 0.0f, 3.5f, 3.0f, 1,
  24.148f, 6.470f, 3.5f, 3.0f, 2,
  21.651f, 12.5f, 3.5f, 3.0f, 3,
  17.678f, 17.678f, 3.5f, 3.0f, 4,
  12.5f, 21.651f, 3.5f, 3.0f, 5,
  6.470f, 24.148f, 3.5f, 3.0f, 6,
  0.0f, 25.0f, 3.5f, 3.0f, 7,
  -6.470f, 24.148f, 3.5f, 3.0f, 8,
  -12.5f, 21.651f, 3.5f, 3.0f, 9,
  -17.678f, 17.678f, 3.5f, 3.0f, 10,
  -21.651f, 12.5f, 3.5f, 3.0f, 11,
  -24.148f, 6.470f, 3.5f, 3.0f, 12,
  -25.0f, 0.0f, 3.5f, 3.0f, 13,
  -24.148f, -6.470f, 3.5f, 3.0f, 14,
  -21.651f, -12.5f, 3.5f, 3.0f, 15,
  -17.678f, -17.678f, 3.5f, 3.0f, 16,
  -12.5f, -21.651f, 3.5f, 3.0f, 17,

  -6.470f, -24.148f, 3.5f, 3.0f, 18,
  0.0f, -25.0f, 3.5f, 3.0f, 19,
  6.470f, -24.148f, 3.5f, 3.0f, 20,
  12.5f, -21.651f, 3.5f, 3.0f, 21,
  17.678f, -17.678f, 3.5f, 3.0f, 22,
  21.651f, -12.5f, 3.5f, 3.0f, 23,
  24.148f, -6.470f, 3.5f, 3.0f, 24,
  0.0f, 0.0f, 3.5f, 3.5f, 0  // Centered mirror ball - Sphere 25
};


/*------------------------------------------------------------------------
  A 3D vector class
------------------------------------------------------------------------*/
struct vec3 {
  float x, y, z;

  // Default constructor using initializer list for default values
  vec3()
    : x(0.0f), y(0.0f), z(0.0f) {}

  // Constructor using initializer list
  vec3(float a, float b, float c)
    : x(a), y(b), z(c) {}

  // Vector addition using member-wise addition
  vec3 operator+(const vec3& v) const {
    return vec3(x + v.x, y + v.y, z + v.z);
  }

  // Vector subtraction using member-wise subtraction
  vec3 operator-(const vec3& v) const {
    return vec3(x - v.x, y - v.y, z - v.z);
  }

  // Vector scaling by a scalar
  vec3 operator*(float s) const {
    return vec3(x * s, y * s, z * s);
  }

  // Dot product (scalar product)
  float operator%(const vec3& v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  // Cross product (vector product)
  vec3 operator^(const vec3& v) const {
    return vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  // Normalized vector (unit vector)
  vec3 operator!() const {
    float lenSquared = x * x + y * y + z * z;
    if (lenSquared == 0.0f) return vec3(0.0f, 0.0f, 0.0f);
    float invLen = 1.0f / std::sqrt(lenSquared);
    return vec3(x * invLen, y * invLen, z * invLen);
  }

  // Compound addition assignment
  void operator+=(const vec3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }

  // Compound multiplication assignment by a scalar
  void operator*=(float s) {
    x *= s;
    y *= s;
    z *= s;
  }
};

// A ray...
struct ray {
  // This occupies 24 bytes - you could only fit 20 of these into
  // a Tiny85 even if you could use the entire RAM (which you can't...)
  vec3 o;  // Origin
  vec3 d;  // Direction
};

/*------------------------------------------------------------------------
  Intersect a ray with the world
  Return 'SKY' if no hit was found but ray goes upward
  Return 'FLOOR' if no hit was found but ray goes downward towards the floor
  Return a material index if a hit was found

  Distance to the hit is returned in 'distance'. 
  The surface normal at the hit is returned in 'normal'
------------------------------------------------------------------------*/
// Values for 'SKY' and 'FLOOR'
static const byte SKY = 255;
static const byte FLOOR = 254;

byte trace(const ray& r, float& distance, vec3& normal) {
  // Pre-calculate the inverse of the ray direction Z component if not zero to avoid division within the loop
  float invDirZ = r.d.z != 0.0f ? 1.0f / r.d.z : 0.0f;

  // Assume we didn't hit anything
  byte result = SKY;
  distance = FLT_MAX;  // Set initial distance to maximum possible float value

  // Check if the ray goes downwards by checking the sign of r.d.z
  if (r.d.z < 0) {
    float floorDistance = -r.o.z / r.d.z;  // Calculate the intersection distance with the floor
    // Check if this is the closest hit so far
    if (floorDistance > 0.01f && floorDistance < distance) {
      // Yes, assume it hits the floor
      distance = floorDistance;
      normal = vec3(0, 0, 1);  // Normal of the floor is upwards
      result = FLOOR;
    }
  }

  vec3 oc = r.o;  // Create a copy of ray origin to modify per sphere

  float sphereData[5];  // Temporary storage for sphere data

  for (byte i = 0; i < NUM_SPHERES; ++i) {
    // Read all sphere data at once
    memcpy_P(sphereData, spheres + (i * 5), sizeof(sphereData));

    // Pre-calculate terms used in intersection test
    float ocX = r.o.x - sphereData[0];
    float ocY = r.o.y - sphereData[1];
    float ocZ = r.o.z - sphereData[2];
    float radiusSquared = sphereData[3] * sphereData[3];

    // Perform the intersection test with pre-calculated terms
    float b = ocX * r.d.x + ocY * r.d.y + ocZ * r.d.z;
    float c = ocX * ocX + ocY * ocY + ocZ * ocZ - radiusSquared;
    float discriminant = b * b - c;

    if (discriminant > 0) {
      discriminant = sqrt(discriminant);  // Only compute the square root if needed
      float t0 = -b - discriminant;
      float t1 = -b + discriminant;

      // Use squared distances for comparison to avoid another square root
      float t = t0 > 0.01f ? t0 : t1 > 0.01f ? t1
                                             : FLT_MAX;
      float squaredDistance = t * t;

      // Update the closest hit distance (compare squared distances)
      if (squaredDistance > 0.0001f && squaredDistance < distance * distance) {
        distance = t;  // We can take the square root now since we know we have the closest distance
        // Update the normal with inlined vector operations
        normal.x = (ocX + r.d.x * t) / sphereData[3];
        normal.y = (ocY + r.d.y * t) / sphereData[3];
        normal.z = (ocZ + r.d.z * t) / sphereData[3];
        result = static_cast<byte>(sphereData[4]);
      }
    }
  }
  return result;
}

float raise(float p, byte n) {
  while (n--) {
    p = p * p;
  }
  return p;
}

/*----------------------------------------------------------
  Small, fast pseudo-random number generator
  
  I found this in a forum and I'm not sure who originally
  wrote it. It works very well though....
 
  If you wrote this then get in touch and I'll put
  your name here. :-)                              FTB.
----------------------------------------------------------*/
byte rngA, rngB, rngC, rngX;
byte randomByte() {
  ++rngX;                              // X is incremented every round and is not affected by any other variable
  rngA = (rngA ^ rngC ^ rngX);         // note the mix of addition and XOR
  rngB = (rngB + rngA);                // And the use of very few instructions
  rngC = (rngC + (rngB >> 1) ^ rngA);  // the right shift is to ensure that high-order bits from B can affect
  return rngC;
}

// A random float in the range [-0.5 ... 0.5]  (more or less)
float randomFloat() {
  char r = char(randomByte());
  return float(r) / 256.0;
}
#define RF randomFloat()
#define SH (RF * shadowRegion)

/*------------------------------------------------------------------------
  Sample the world and return the pixel color for a ray
------------------------------------------------------------------------*/
float sample(ray& r, vec3& color) {
  // See if the ray hits anything in the world
  float t;
  vec3& n = color;  // RAM is tight, use 'color' as temp workspace
  const byte hit = trace(r, t, n);

  // Did we hit anything
  if (hit == SKY) {
    // Generate a sky color if the ray goes upwards without hitting anything
    color = vec3(0.1f, 0.0f, 0.3f) + vec3(.7f, .2f, 0.5f) * raise(1.0 - r.d.z, 2);
    return 0.0;
  }

  // New ray origin
  r.o += r.d * t;

  // Half vector
  const vec3 half = !(r.d + n * ((n % r.d) * -2));

  // Vector that points towards the light
  r.d = vec3(9 + SH, 6 + SH, 16);  // Where the light is
  r.d = !(r.d - r.o);              // Normalized light vector

  // Lambertian factor
  float d = r.d % n;  // Light vector % surface normal

  // See if we're in shadow
  if ((d < 0) or (trace(r, t, n) != SKY)) {
    d = 0;
  }

  // Did we hit the floor?
  if (hit == FLOOR) {
    // Yes, generate a floor color
    d = (d * 0.2) + 0.1;
    t = d * 3.0;                                                   // d=dark, t=light
    color = vec3(t, t, t);                                         // Assume grey color
    t = 1.0f / 5.0f;                                               // Floor tiles are 5m across
                                                                   //    int fx = int(ceil(r.o.x*t));
                                                                   //    int fy = int(ceil(r.o.y*t));
                                                                   //    bool dark = ((fx+fy)&1)!=0;  // Light or dark color?
    bool dark = (((int)(ceil(r.o.x * t) + ceil(r.o.y * t))) & 1);  // Light or dark color? -> fix for AVR compiler
    if (dark) { color.y = color.z = d; }                           // g+b => dark => 'red'
    return 0;
  }

  // No, we hit the scene, read material color from progmem
  const float* mat = materials + (hit * 4);
  color.x = pgm_read_float(mat++);
  color.y = pgm_read_float(mat++);
  color.z = pgm_read_float(mat++);

  // Specular light in 't'
  t = d;
  if (t > 0) {
    t = raise(r.d % half, 5);
  }

  // Calculate total color using diffuse and specular components
  color *= d * d + ambient;  // Ambient+diffuse
  color += vec3(t, t, t);    // Specular

  // We need to trace a reflection ray...need to modify 'r' for the recursion
  r.d = half;
  return pgm_read_float(mat);  // Reflectivity of this material
}

/*------------------------------------------------------------------------
  Raytrace the entire image
------------------------------------------------------------------------*/
void doRaytrace(int raysPerPixel = 8, int dw = 240, int dh = 320, int q = 1) {
//void doRaytraceOptimized(int raysPerPixel = 8, int dw = 240, int dh = 320, int q = 1) {
    int dw2 = dw / 2;
    int dh2 = dh / 2;
    const float pixel = fov / float(dh2);  // Size of one pixel on screen

    // Calculate new camera position based on orbit_angle_degrees
    float orbit_angle_radians = radians(orbit_angle_degrees);
    float new_cameraX = targetX + orbit_radius * cos(orbit_angle_radians);
    float new_cameraY = targetY + orbit_radius * sin(orbit_angle_radians);
    vec3 camera = vec3(new_cameraX, new_cameraY, cameraZ);
    const vec3 target = vec3(targetX, targetY, targetZ);

    unsigned long t = millis();

    for (int y = 0; y < dh; y += q) {
        for (int x = 0; x < dw; x += q) {
            vec3 acc(0, 0, 0);  // Color accumulator
            for (int p = raysPerPixel; p--;) {
                ray r;
                vec3 temp;
                float xpos = float(x - dw2), ypos = float(dh2 - y);
                if (raysPerPixel > 1) {
                    xpos += RF;
                    ypos += RF;
                }

                temp = !(target - camera);
                vec3& right = r.o;
                right = !(temp ^ vec3(0, 0, 1));
                vec3& up = r.d;
                up = !(right ^ temp);
                r.d = !(temp + ((right * xpos) + (up * ypos)) * pixel);
                r.o = camera;

                float reflect1 = sample(r, temp);
                acc += temp;
                if (reflect1 > 0) {
                    float reflect2 = sample(r, temp);
                    acc += temp * reflect1;
                    if (reflect2 > 0) {
                        sample(r, temp);
                        acc += temp * (reflect1 * reflect2);
                    }
                }
            }
            acc = acc * (255.0f / raysPerPixel);
            uint8_t r = (uint8_t)min(acc.x, 255.0f);
            uint8_t g = (uint8_t)min(acc.y, 255.0f);
            uint8_t b = (uint8_t)min(acc.z, 255.0f);
            if (q == 1) M5.Lcd.drawPixel(x, y, RGBTO565(r, g, b));
            else M5.Lcd.fillRect(x, y, q, q, RGBTO565(r, g, b));
        }
    }
    orbit_angle_degrees += angle_increment;
    if (orbit_angle_degrees >= 360.0) {
        orbit_angle_degrees -= 360.0;  // Wrap angle to maintain the continuous loop
    }
}