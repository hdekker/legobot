#include <map>
#include <algorithm> // sort
#include <iostream>
#include <vector>

#include "lms2012.h"
#include "unistd.h"
#include "fastmath.h"
#include "common.h"
#include "command.h"
#include "motors.h"
#include "robot.h"
#include "sensors.h"

#define MAX(i, j)  ((i)>(j))?(i):(j)
#define MIN(i, j)  ((i)<(j))?(i):(j)

#define W                   (2000)    // width of board [mm]
#define H                   (2500)    // heigth of board [mm]
#define TOO_CLOSE_TOO_DETECT_BORDER  (100) // Cannot see border when too close
#define TOO_FAR_TOO_DETECT_BORDER   (1000) // Cannot see border when too far
#define TARGET_X            (1000)
#define TARGET_Y            (1510)
#define TARGET_R0            (390)    // radius of target [mm]
#define TARGET_R1            (545)    // radius of target outer ring [mm]
#define T_D                   (40)    // tolerance of distance measurement [mm]
#define T_A                 (2000)    // ultrasone detection angle [0..65535] (4000 is around 20 degrees 360 scale )
#define N_PARTICLES          (500)    // Number of particles

using namespace std;

// Measurements as seen from robot. 
// angle -90 = right side of robot
// angle 0   = front side of robot
// angle +90 = left side of robot

struct measurement
{
  int a; // angle [deg] (0..N_DEGREES)
  int d; // distance [mm]
};
typedef vector<measurement> Measurements;

struct position_t
{
  int x; // pos [mm]
  int y; // pos [mm]
  int a; // angle [deg] (0..N_DEGREES)
};




class CParticle
{
  position_t p;
  int importance;
public:
  CParticle();
  int sense(const Measurements& m);
  void set_random();
  void set_random(CParticle& part);
  //void set_random(position_t pos);
  void move(int angle, int distance);
  void set_position(int x, int y, int a);
  position_t get_position();
  int get_importance();
  void print();
  bool operator<(const CParticle& other) const
  {
    // Sort descending
    return this->importance > other.importance;
  }
  bool is_valid_weight();
private:
  bool is_valid_position(position_t pos);
};

typedef vector<CParticle> Particles;


CParticle::CParticle()
{
  importance = 0;
}


bool CParticle::is_valid_position(position_t pos)
{
  // r is distance from target
  int r = fastmath_sqrt(((TARGET_X - pos.x) * (TARGET_X - pos.x)) + ((TARGET_Y - pos.y) * (TARGET_Y - pos.y)));
  bool is_valid = ((pos.x >= 0) && (pos.x < W) && (pos.y >= 0) && (pos.y < H) && (r > TARGET_R0));
  //std::cout << "x=" << pos.x << " y=" << pos.y << " a=" << pos.a << " is_valid=" << is_valid << "\n";
  return is_valid;
}

int CParticle::sense(const Measurements& measurements)
{
  importance = 0;
  //std::cout << "\n\n";
  //print();
  
  for (const measurement& m : measurements)
  {
    //std::cout << "\nm.a=" << m.a << " m.d=" << m.d << "\n";

    // Take ultrasonic angle width into consideration
    vector<int> angles;
    //angles.push_back(p.a + m.a - T_A);    // right tolerance
    //angles.push_back(p.a + m.a - T_A/2);  // right tolerance
    angles.push_back(p.a + m.a);          // straight
    //angles.push_back(p.a + m.a + T_A/2);  // left tolerance
    //angles.push_back(p.a + m.a + T_A);    // left tolerance
    for (int& measure_angle : angles)
    {
      position_t detected;
      detected.a = POS_DEGREES(measure_angle + fastmath_gaussrandom(0, UINT16_2_PI/100));
      detected.x = p.x + ((m.d * fastmath_cos(detected.a)) >> TRI_SHIFT) + fastmath_gaussrandom(0, T_D/4);
      detected.y = p.y + ((m.d * fastmath_sin(detected.a)) >> TRI_SHIFT) + fastmath_gaussrandom(0, T_D/4);
      int distance = 0;
      int angle = 0;
      //std::cout << "a=" << a << " x=" << x << " y=" << y << "\n";
      
      // Horizontal left wall detection
      distance = MAX(T_D - abs(detected.x), 0);
      angle = MAX(T_A - abs(UINT16_PI - detected.a), 0);
      importance += fastmath_log2(distance * angle);
      //std::cout << "left wall: d=" << distance << " a=" << angle << "\n";
      
      // Horizontal right wall detection
      distance = MAX(T_D - abs(W - detected.x), 0);
      angle = MAX(T_A - abs(detected.a), 0);
      importance += fastmath_log2(distance * angle);
      //std::cout << "right wall: d=" << distance << " a=" << angle << "\n";
      
      // Vertical bottom wall detection
      distance = MAX(T_D - abs(detected.y), 0);
      angle = MAX(T_A - abs(UINT16_H_PI*3 - detected.a), 0);
      importance += fastmath_log2(distance * angle);
      //std::cout << "bottom wall: d=" << distance << " a=" << angle << "\n";
      
      // Vertical top wall detection
      distance = MAX(T_D - abs(H - detected.y), 0);
      angle = MAX(T_A - abs(UINT16_H_PI - detected.a), 0);
      importance += fastmath_log2(distance * angle);
      //std::cout << "top wall: d=" << distance << " a=" << angle << "\n";
      
      // Target detection (inner and outer ring)
      vector<int> radi;
      radi.push_back(TARGET_R0);
      radi.push_back(TARGET_R1);
      int delta_x = TARGET_X - detected.x;
      int delta_y = TARGET_Y - detected.y;
      int dc = fastmath_sqrt(delta_x * delta_x + delta_y * delta_y);
      int ac = fastmath_atan2(delta_y, delta_x);
      int as = (UINT16_H_PI + detected.a - ac) % (UINT16_H_PI*2) - UINT16_H_PI; // To make sure 32000 and 768 result both in 768
      //std::cout << "dc=" << dc << " as=" << as << "\n";
      //std::cout << "at=" << at << " ac=" << ac << "\n";
      for (int& radius : radi)
      {
        distance = MAX(T_D - abs(dc - radius), 0);
        angle = MAX(T_A - abs(as), 0);
        importance += fastmath_log2(distance * angle);
        //std::cout << "center: d=" << distance << " a=" << angle << "\n";
      }
    }
  }
  //std::cout << "weight=" << weight << "\n";
  //avg_weight = ((avg_weight == 0) ? (weight * UINT16_PI/2) : ((avg_weight + weight) / 2)) >> TRI_SHIFT;
  return importance;
}

void CParticle::set_random()
{
  // Fill only one quadrant (because all quadrants will be the same)
  do
  {
    p.x = fastmath_gaussrandom(500, 250);
    p.y = fastmath_gaussrandom(700, 400);
    p.a = POS_DEGREES(fastmath_gaussrandom(0, UINT16_PI));
  } while (!is_valid_position(p));
}

void CParticle::set_random(CParticle& part)
{
  // Set new position near a successfull position
  do
  {
    p.x = fastmath_gaussrandom(part.p.x, 200);
    p.y = fastmath_gaussrandom(part.p.y, 200);
    p.a = POS_DEGREES(fastmath_gaussrandom(part.p.a, 2000)); // 1000 is around 1/64th degree
  } while (!is_valid_position(p));
}

/*
void CParticle::set_random(const position_t pos)
{
  // Set new position near a successfull position
  do
  {
    p.x = fastmath_gaussrandom(pos.x, W/8); // + delta_position_mm(gen);// fmod(rand(), W/10);
    p.y = fastmath_gaussrandom(pos.y, H/8); // + delta_position_mm(gen);// fmod(rand(), H/10);
    p.a = POS_DEGREES(fastmath_gaussrandom(pos.a, 1000));
  } while (!is_valid_position(p));
}
*/

void CParticle::move(int angle, int distance)
{
  //std::cout << "angle=" << angle << " distance=" << distance << "\n";
  //print();
  p.a = POS_DEGREES(p.a + angle); //+ move_noise_rad(gen), 2 * UINT16_PI);
  p.x += (distance * fastmath_cos(p.a)) >> TRI_SHIFT;// + move_noise_mm(gen);
  p.y += (distance * fastmath_sin(p.a)) >> TRI_SHIFT;// + move_noise_mm(gen);
  //print();
}

position_t CParticle::get_position()
{
  return p;
}

int CParticle::get_importance()
{
  return importance;
}

void CParticle::set_position(int x, int y, int a)
{
  p.x = x;
  p.y = y;
  p.a = a;
}

bool CParticle::is_valid_weight()
{
  return importance > 1;
}

void CParticle::print()
{
  std::cout << "x=" << p.x << " y=" << p.y << " a=" << p.a << " w=" << importance << "\n";
}






class CParticles
{
  Particles particles_1;
  Particles particles_2;
  Particles* p_particles;
  position_t previous_position;
  
public:
  CParticles();
  bool sense(const Measurements& meas, position_t& position_mean, position_t& position_deviation);
  void move(int angle, int distance);
  void reset();
private:
  void fill_particles(int i=0);
};

CParticles::CParticles()
{
  previous_position = { 0, 0, 0 };
  p_particles = &particles_1;
  particles_1.resize(N_PARTICLES);
  particles_2.resize(N_PARTICLES);
  fill_particles();
}

void CParticles::fill_particles(int i)
{
  CParticle initial_particle;
  initial_particle.set_position(400, 800, DEG2UINT16(45));
  
  
  for (CParticle& part : *p_particles) part.set_random(initial_particle); // initial_particle);
  // Initial guess
  if (i==0)
  {
    (*p_particles)[0].set_position(450, 450, UINT16_PI/4);
    (*p_particles)[1].set_position(450, 1550, UINT16_PI * 7 / 8);
    (*p_particles)[2].set_position(1550, 1550, UINT16_PI * 5 / 8);
    (*p_particles)[3].set_position(1550, 450, UINT16_PI * 3 / 4);
    (*p_particles)[4].set_position(230, 1000, UINT16_PI * 0);
    (*p_particles)[5].set_position(1000, 1770, UINT16_PI * 3 / 2);
    (*p_particles)[6].set_position(1770, 1000, UINT16_PI * 1);
    (*p_particles)[7].set_position(1000, 230, UINT16_PI / 2);
  }
  //for (CParticle& part : **p_particles) part.print();
}

void CParticles::move(int angle, int distance)
{
  for (CParticle& part : *p_particles) part.move(angle, distance);
}

void CParticles::reset()
{
  for (CParticle& part : *p_particles) part.set_random();
}

bool CParticles::sense(const Measurements& meas, position_t& position_mean, position_t& position_deviation)
{
  bool particles_reset = false;
  position_t position_square_sum;
  
  // Always to other array, compared with 'p_particles' itself
  Particles* p_tmp_particles = (p_particles == &particles_1) ? &particles_2 : &particles_1;

  // Sense also returns it's importance (i.e. how well it fits with reality)
  int sum_importance = 0;
  while (sum_importance == 0)
  {
    for (CParticle& part : *p_particles)
    {
      sum_importance += part.sense(meas);
    }
    if (sum_importance == 0)
    {
      std::cout << "WARNING: All particles at wrong positions\n";
      particles_reset = true;
      for (CParticle& part : *p_particles) part.set_random();
    }
  }
  
  // The more importance the more particles are stored in weighted_particles vector,
  // and the more chance this particle produces sibblings.
  std::vector<CParticle*> weighted_particles;
  std::vector<int> weighted_x;
  std::vector<int> weighted_y;
  
  for (CParticle& part : *p_particles)
  {
    int weight = (part.get_importance() * N_PARTICLES) / sum_importance;
    position_t pos = part.get_position();
    for (int w=0; w<weight; w++)
    {
      weighted_particles.push_back(&part);
      weighted_x.push_back(pos.x);
      weighted_y.push_back(pos.y);
    }
  }
  
  // X-axis statistics
  position_mean.x = std::accumulate(weighted_x.begin(), weighted_x.end(), 0) / weighted_x.size();
  position_square_sum.x = std::inner_product(weighted_x.begin(), weighted_x.end(), weighted_x.begin(), 0);
  position_deviation.x = fastmath_sqrt(position_square_sum.x / weighted_x.size() - position_mean.x * position_mean.x);

  // Y-axis statistics
  position_mean.y = std::accumulate(weighted_y.begin(), weighted_y.end(), 0) / weighted_y.size();
  position_square_sum.y = std::inner_product(weighted_y.begin(), weighted_y.end(), weighted_y.begin(), 0);
  position_deviation.y = fastmath_sqrt(position_square_sum.y / weighted_y.size() - position_mean.y * position_mean.y);
  
  // Rotation difficult to perform analyze, seems to be too random (because overlap 360->0 degrees?)
  position_mean.a = fastmath_atan2((position_mean.y - previous_position.y), (position_mean.x - previous_position.x));
  position_square_sum.a = 0;
  position_deviation.a = 0;;
  
  // Create new set of particles
  for (int i=0; i<N_PARTICLES; i++)
  {
    int index = rand() % weighted_particles.size();
    (*p_tmp_particles)[i].set_random(*weighted_particles[index]);
  }
  
  // Point to the new vectors
  previous_position = position_mean;
  p_particles = p_tmp_particles;
  
  return particles_reset;
}


class CRobotSimulator
{
  position_t p;
public:
  CRobotSimulator();
  void print();
  void measure(Measurements& m);
  void move(int distance);
  void turn(int angle);
  void random();
  int calc_distance(int angle);
  void set_position(int x, int y, int a);
  void self_test();
};

CRobotSimulator::CRobotSimulator()
{
  p.x = 400;
  p.y = 300;
  p.a = 0;
}

void CRobotSimulator::print()
{
  std::cout << "Robot position: x=" << p.x << " y=" << p.y << " a=" << p.a << "\n";
}

void CRobotSimulator::set_position(int x, int y, int a)
{
  p.x = x;
  p.y = y;
  p.a = POS_DEGREES(a);
}

void CRobotSimulator::move(int distance)
{
  p.x += (distance * fastmath_cos(p.a)) >> TRI_SHIFT;
  p.y += (distance * fastmath_sin(p.a)) >> TRI_SHIFT;
}

void CRobotSimulator::turn(int angle)
{
  p.a = POS_DEGREES(p.a + angle);
}

void CRobotSimulator::random()
{
  if (rand() % 10 == 0)
  {
    turn(-UINT16_PI/20 + fmod(rand(), UINT16_PI/10));
  }
  else
  {
    move(5);
  }
}

void CRobotSimulator::measure(Measurements& m)
{
  m.resize(3);
  m[0] = { DEG2UINT16(-90), calc_distance(DEG2UINT16(-90)) };
  //m[1] = { DEG2UINT16(0), calc_distance(DEG2UINT16(0)) };
  //m[2] = { DEG2UINT16(90), calc_distance(DEG2UINT16(90)) };
  //m[3] = { DEG2UINT16(180), calc_distance(DEG2UINT16(180)) };
  //m[4] = { DEG2UINT16(270), calc_distance(DEG2UINT16(270)) };
  //m[5] = { DEG2UINT16(360), calc_distance(DEG2UINT16(360)) };
  //m[6] = { DEG2UINT16(45), calc_distance(DEG2UINT16(45)) };
  //m[7] = { DEG2UINT16(135), calc_distance(DEG2UINT16(135)) };
  //m[8] = { DEG2UINT16(225), calc_distance(DEG2UINT16(225)) };
  //m[9] = { DEG2UINT16(315), calc_distance(DEG2UINT16(315)) };
}

int CRobotSimulator::calc_distance(int angle)
{
  int distance = W + H; // No existing distance
  int a = POS_DEGREES(p.a + angle);
  int rc = fastmath_tan(a); // A little bit inaccurate when only moving in Y direction!
  //std::cout << "rc=" << rc << "\n";
  bool x_pos = ((a>=0 && a<=UINT16_H_PI) || (a>=UINT16_H_PI*3 && a<=UINT16_2_PI));
  bool y_pos = (a>=0 && a<=UINT16_PI);
  int distance_to_wall = (x_pos) ? W - p.x : -p.x;
  int x = (x_pos) ? W : 0;
  int y = p.y + ((rc * distance_to_wall) >> TRI_SHIFT);
  if (y>=0 && y<=H)
  {
    // Line hitting left or right walls
    distance = fastmath_sqrt((y-p.y) * (y-p.y) + distance_to_wall * distance_to_wall);
    //std::cout << "vertical walls distance=" << distance << "\n";
    // In negative direction we can still see the maze
    distance_to_wall = (distance_to_wall<TOO_CLOSE_TOO_DETECT_BORDER) ? W*2 : ((distance_to_wall>TOO_FAR_TOO_DETECT_BORDER) ? W*2 : distance_to_wall);
  }
  else 
  {
    // Line hitting upper or bottom walls
    distance_to_wall = (y_pos) ? H - p.y : -p.y;
    x = (distance_to_wall << TRI_SHIFT) / rc; 
    distance = fastmath_sqrt(x * x + distance_to_wall * distance_to_wall);
    x = p.x + x;
    y = (y_pos) ? H : 0;
    // std::cout << "horizontal walls distance=" << distance << "\n";
    if (y_pos) 
    {
      // In negative direction we can still see the maze
      distance_to_wall = (distance_to_wall<TOO_CLOSE_TOO_DETECT_BORDER) ? H*2 : ((distance_to_wall>TOO_FAR_TOO_DETECT_BORDER) ? H*2 : distance_to_wall);
    }
  }
  
  /////////////////////
  // Q2 Q3 
  // Q1 Q4
  ////////////////////
  bool can_see_target = ((p.x < TARGET_X and p.y < TARGET_Y and a > 0 and a < UINT16_H_PI) ||              // Q1
                         (p.x < TARGET_X and p.y > TARGET_Y and a > UINT16_H_PI*3 and a < UINT16_2_PI) ||  // Q2
                         (p.x > TARGET_X and p.y > TARGET_Y and a > UINT16_PI and a < UINT16_H_PI*3) ||    // Q3
                         (p.x > TARGET_X and p.y < TARGET_Y and a > UINT16_H_PI and a < UINT16_PI));       // Q4
  
  if (can_see_target)
  {
    // Now check collision with target start=(p.x, p.y) and end=(x, y)
    //std::cout << "p(x,y)=(" << p.x << ", " << p.y << ")\n";
    //std::cout << " (x,y)=(" << x << ", " << y << ")\n";
    
    // compute the euclidean distance between start (A) and end (B)
    int lab = fastmath_sqrt( (x-p.x) * (x-p.x) + (y-p.y) * (y-p.y) );
    //std::cout << "lab=(" << lab << ")\n";
    
    // compute the direction vector D from A to B
    int dx = ((x-p.x) << TRI_SHIFT) / lab;
    int dy = ((y-p.y) << TRI_SHIFT) / lab;
    //std::cout << "d(x,y)=(" << dx << ", " << dy << ")\n";
    // Now the line equation is x = dx*t + ax, y = dy*t + ay with 0 <= t <= 1.

    // compute the value t of the closest point to the circle center (Cx, Cy)
    int t = ((dx * (TARGET_X - p.x)) >> TRI_SHIFT) + ((dy * (TARGET_Y - p.y)) >> TRI_SHIFT);
    //std::cout << "t=(" << t << ")\n";
    
    // This is the projection of C on the line from A to B.

    // compute the coordinates of the point E on line and closest to C
    int ex = ((t * dx) >> TRI_SHIFT) + p.x;
    int ey = ((t * dy) >> TRI_SHIFT) + p.y;
    //std::cout << "e(x,y)=(" << ex << ", " << ey << ")\n";
    
    // compute the euclidean distance from E to C
    int lec = fastmath_sqrt( (ex-TARGET_X) * (ex-TARGET_X) + (ey-TARGET_Y) * (ey-TARGET_Y) );
    //std::cout << "lec(" << lec << ")\n";
    
    // test if the line intersects the circle
    if (lec < TARGET_R0)
    {
        // compute distance from t to circle intersection point
        int dt = fastmath_sqrt( TARGET_R0*TARGET_R0 - lec*lec);
        //std::cout << "dt(" << dt << ")\n";
        
        // compute first intersection point
        int fx = (((t-dt)*dx) >> TRI_SHIFT) + p.x;
        int fy = (((t-dt)*dy) >> TRI_SHIFT) + p.y;
        //std::cout << "f(x,y)=(" << fx << ", " << fy << ")\n";
        
        // compute second intersection point
        int gx = (((t+dt)*dx) >> TRI_SHIFT) + p.x;
        int gy = (((t+dt)*dy) >> TRI_SHIFT) + p.y;
        //std::cout << "g(x,y)=(" << gx << ", " << gy << ")\n";

        // compute distances from robot        
        int df = fastmath_sqrt((fx-p.x)*(fx-p.x) + (fy-p.y)*(fy-p.y));
        int dg = fastmath_sqrt((gx-p.x)*(gx-p.x) + (gy-p.y)*(gy-p.y));
        //std::cout << "d(f,g)=(" << df << ", " << dg << ")\n";
        
        distance = MIN(df, dg);
        //std::cout << "target at " << distance << "mm distance\n";
    }
  }
  
  //std::cout << "distance[" << angle << "]=" << distance << "  wall_impact(x,y)=(" << x << ", " << y << ")\n";
  return distance;
}
 
 
void CRobotSimulator::self_test()
{
  p.x = 400;
  p.y = 300;
  p.a = 0;
  calc_distance(0);
  calc_distance(UINT16_PI/6);
  calc_distance(UINT16_PI/4);
  calc_distance(UINT16_PI/4 + UINT16_PI/6);
  calc_distance(UINT16_PI/2);
  calc_distance(UINT16_PI/2 + UINT16_PI/6);
  calc_distance(UINT16_PI/2 + UINT16_PI/4);
  calc_distance(UINT16_PI/2 + UINT16_PI/4 + UINT16_PI/6);
  calc_distance(UINT16_PI);
  calc_distance(UINT16_PI + UINT16_PI/6);
  calc_distance(UINT16_PI + UINT16_PI/4);
  calc_distance(UINT16_PI + UINT16_PI/4 + UINT16_PI/6);
  calc_distance(UINT16_PI + UINT16_PI/2);
  calc_distance(UINT16_PI + UINT16_PI/2 + UINT16_PI/6);
  calc_distance(UINT16_PI + UINT16_PI/2 + UINT16_PI/4);
  calc_distance(UINT16_PI + UINT16_PI/2 + UINT16_PI/4 + UINT16_PI/6);
  calc_distance(UINT16_PI + UINT16_PI);
  p.x = 1600;
  p.y = 800;
  p.a = UINT16_PI;
  calc_distance(-UINT16_PI/4);
  p.x = 1600;
  p.y = 1700;
  p.a = UINT16_PI;
  calc_distance(UINT16_PI/4);
  p.x = 400;
  p.y = 1700;
  p.a = UINT16_PI/2;
  calc_distance(UINT16_PI + UINT16_PI/4);
  // To default
  p.x = 400;
  p.y = 300;
  p.a = 0;
}

std::vector<position_t> positions_to_do = 
{ 
  {  400, 1000, DEG2UINT16(225) },
  {  250, 1500, DEG2UINT16(180) },
  {  400, 2000, DEG2UINT16(135) },
  { 1000, 2200, DEG2UINT16(90)  },
  { 1600, 2000, DEG2UINT16(45)  },
  { 1750, 1500, DEG2UINT16(0)   },
  { 1600, 1000, DEG2UINT16(315) },
  { 1000,  700, DEG2UINT16(270) },
  {  400,  400, DEG2UINT16(0)   },
};


int old_main(int argc, const char **argv, char * const *envp)
{
  
  CRobotSimulator* rsim = new CRobotSimulator();
  rsim->self_test();
  
  CParticles particles;
  
  Measurements meas;
  
  rsim->set_position(300, 800, DEG2UINT16(180));
  
  for (int i=0; i<100; i++)
  {
    std::cout << "\nNext step\n";
    if (i%15 == 0)
    {
      rsim->turn(DEG2UINT16(-90));
      rsim->print();
      particles.move(DEG2UINT16(-90), 0);
    }
    else
    {
      rsim->move(100);
      rsim->print();
      particles.move(DEG2UINT16(0), 100);
    }
    rsim->measure(meas);
    position_t pos;
    position_t dev;
    particles.sense(meas, pos, dev);
    //std::cout << "deviation=" << "x=" << dev.x << " y=" << dev.y << " a=" << dev.a << "\n";
    std::cout << "Estimated pos.:" << " x=" << pos.x << " y=" << pos.y << " a=" << pos.a << "\n";
    // If standard deviation too big, then reavaluate sensor data.
    if (dev.x + dev.y > 350)
    {
      //std::cout << "\nReset particles\n";
      particles.reset();
      particles.sense(meas, pos, dev);
      //std::cout << "deviation=" << "x=" << dev.x << " y=" << dev.y << " a=" << dev.a << "\n";
      std::cout << "Re-estimated. :" << " x=" << pos.x << " y=" << pos.y << " a=" << pos.a << "\n";
    }
    sleep(1);
  }
  
  return 0;
}


position_t diff_position(position_t p0, position_t p1)
{
  position_t diff_pos;
  diff_pos.x = p1.x - p0.x;
  diff_pos.y = p1.y - p0.y;
  diff_pos.a = POS_DEGREES(p1.a - p0.a);
  return diff_pos;
}



bool equal_pos(position_t p0, position_t p1)
{
  position_t pos = diff_position(p0, p1);
  return (abs(pos.x) < 50) && (abs(pos.y) < 50) && (abs(pos.a) < 2000);
}

void move_to_position(position_t end_pos, bool* keep_running, CParticles& particles, Measurements& measurements)
{
  // static position_t cur_pos = { 300, 500, DEG2UINT16(45) };
  position_t cur_pos;
  position_t dev_pos;

  // Get current situation  
  measurements.resize(2);
  measurements[0] = { DEG2UINT16(-90), command_get_right_distance_mm() };
  measurements[1] = { DEG2UINT16(  0), command_get_forward_distance_mm() };
  particles.sense(measurements, cur_pos, dev_pos);
  
  while ((*keep_running) && (!equal_pos(cur_pos, end_pos)))
  {
    // Get difference between estimated position and end position
    position_t diff_pos = diff_position(cur_pos, end_pos);
    
    // Turn
    command_turn_angle(UINT162DEG(diff_pos.a));
    if (*keep_running)
    {
      particles.move(diff_pos.a, 0);
      measurements[0] = { DEG2UINT16(-90), command_get_right_distance_mm() };
      measurements[1] = { DEG2UINT16(  0), command_get_forward_distance_mm() };
      particles.sense(measurements, cur_pos, dev_pos);
    }
    // Move forward
    command_move_distance(20);
    if (*keep_running)
    {
      particles.move(0, 20);
      measurements[0] = { DEG2UINT16(-90), command_get_right_distance_mm() };
      measurements[1] = { DEG2UINT16(  0), command_get_forward_distance_mm() };
      particles.sense(measurements, cur_pos, dev_pos);
    }
  }
}


void ball_execute_old(bool *keep_running)
{
  CParticles particles;
  Measurements meas;
  
  for (position_t& pos: positions_to_do)
  {  
      std::cout << "Next position...\n";
      move_to_position(pos, keep_running, particles, meas);
      if (not *keep_running) break;
  }
  
  printf("Stopping...\n");
  command_move_stop();
};

class CThrower
{
  bool is_down = false;
  bool is_half_down = false;
  bool is_up = false;
  
private:
  void calibrate()
  {
    printf("Calibrate throw engine start\n");
    motors_set_speed(ROBOT_BALL_THROW_PORT, 30);
    SBYTE motor_speeds[4];
    for (int i=0; i<4; i++) motor_speeds[i] = 1;
    int i = 0;
    do
    {
      sleep_ms(100);
      motor_speeds[(i++)%4] = motors_get_motor_speed(ROBOT_BALL_THROW_PORT);
      printf("%d %d %d %d\n", motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
    } while ((abs(motor_speeds[0]) + abs(motor_speeds[1]) + abs(motor_speeds[2]) + abs(motor_speeds[3])) > 0);
    motors_reset_angle(ROBOT_BALL_THROW_PORT);
    motors_stop(ROBOT_BALL_THROW_PORT);
    is_down = false;
    is_half_down = false;
    is_up = false;
    printf("Calibrate throw engine done\n");
  }
  
public:
  
  void up()
  {
    if ((not is_up) and (not is_down) and (not is_half_down))
    {
      calibrate();
    }
    
    if (not is_up)
    {
      printf("Throw engine UP start\n");
      int setpoint_angle = (is_down) ? 180 : -180;
      motors_move_to_angle(ROBOT_BALL_THROW_PORT, 30, setpoint_angle, 1);
      printf("Throw engine UP done\n");
    }
    
    is_up = true;
    is_down = false;
    is_half_down = false;
  }
  
  void down()
  {
    if ((not is_up) and (not is_down) and (not is_half_down))
    {
      calibrate();
    }
    
    if (not is_down)
    {
      printf("Throw engine DOWN start\n");
      int setpoint_angle = (is_half_down) ? -350 : -360;
      motors_move_to_angle(ROBOT_BALL_THROW_PORT, 30, setpoint_angle, 1);
      printf("Throw engine DOWN done\n");
    }
    
    is_up = false;
    is_down = true;
    is_half_down = false;
  }
  
  void half_down()
  {
    if ((not is_up) and (not is_down) and (not is_half_down))
    {
      calibrate();
    }
    
    if (not is_half_down)
    {
      printf("Throw engine HALF_DOWN start\n");
      int setpoint_angle = (is_down) ? 60 : -310;
      motors_move_to_angle(ROBOT_BALL_THROW_PORT, 15, setpoint_angle, 1);
      printf("Throw engine HALF_DOWN done\n");
    }
    
    is_up = false;
    is_down = false;
    is_half_down = true;
  }
  
  void shoot()
  {
    printf("Throw engine SHOOT start\n");
    down();
    printf("SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOT");
    motors_set_speed(ROBOT_BALL_THROW_PORT, -100);
    sleep_ms(200);
    motors_stop(ROBOT_BALL_THROW_PORT);
    is_up = false;
    is_down = false;
    is_half_down = false;
    printf("Throw engine SHOOT done\n");
  }
};




class CCatcher
{
  bool is_open = false;
  bool is_closed = false;
  
public:
  
  bool open()
  {
    printf("Catch engine OPEN start\n");
    bool is_ok = true;
    if (not is_open)
    {
      motors_set_speed(ROBOT_BALL_CATCH_PORT, 30);
      SBYTE motor_speeds[4];
      for (int i=0; i<4; i++) motor_speeds[i] = 1;
      int i = 0;
      int current_angle = motors_get_angle(ROBOT_BALL_CATCH_PORT);
      do
      {
        sleep_ms(100);
        motor_speeds[(i++)%4] = motors_get_motor_speed(ROBOT_BALL_CATCH_PORT);
        printf("CCatcher.open %d %d %d %d\n", motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
      } while ((abs(motor_speeds[0]) + abs(motor_speeds[1]) + abs(motor_speeds[2]) + abs(motor_speeds[3])) > 0);
      int reached_angle = motors_get_angle(ROBOT_BALL_CATCH_PORT);
      motors_reset_angle(ROBOT_BALL_CATCH_PORT);
      motors_stop(ROBOT_BALL_CATCH_PORT);
      // is_ok also true when started with half open arms
      is_ok = (is_closed) ? (abs(current_angle - reached_angle) > 170) : true;
    }
    is_open = true;
    is_closed = false;
    printf("Catch engine OPEN result %d\n", is_ok);
    return is_ok;
  }
  
  bool close()
  {
    bool is_ok = true;
    printf("Catch engine CLOSE start\n");
    
    if (not is_open)
    {
      is_ok = open();
    }
    
    if (is_ok)
    {
      int current_angle = motors_get_angle(ROBOT_BALL_CATCH_PORT);
      motors_move_to_angle(ROBOT_BALL_CATCH_PORT, 30, -180, 1);
      int reached_angle = motors_get_angle(ROBOT_BALL_CATCH_PORT);
      if (abs(reached_angle - current_angle) > 170)
      {
        is_open = false;
        is_closed = true;
      }
      else
      {
        is_ok = false;
      }
    }
    printf("Catch engine CLOSE result %d\n", is_ok);
    return is_ok;
  }
};



class CBallHandler: public CThrower, public CCatcher
{
  
public:
  CBallHandler()
  {
    open();
    up();
    down();
    close();
  }
  
  bool trycatch(COLOR_CODE& color)
  {
    printf("trycatch start\n");
    color = COLOR_CODE_TRANSPARENT;
    bool success = false;
    down();
    for (int i=0; i<3 and not success; i++)
    {
      success = open();
      success = close() and success;
      color = sensors_get_color(ROBOT_COLOR_SENSOR_PORT);
      success = success and (color==COLOR_CODE_BLUE or color==COLOR_CODE_GREEN or 
                             color==COLOR_CODE_YELLOW or color==COLOR_CODE_RED);
    }
    printf("trycatch done (success=%d, color=%d)\n", success, color);
    return success;
  }
};




class CWheelHandler
{
private:
  void move_until_collision()
  {
    printf("move_until_collision start\n");
    time_t starttime = time(NULL);
    time_t currenttime;
    int elapsed = 0;
    int distance = 0;
    int velocity = -80;
    int actual_velocity_left = 0;
    int actual_velocity_right = 0;
    motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, velocity * 11 / 10);
    motors_set_speed(ROBOT_WHEEL_LEFT_PORT, velocity);
    do
    {
      sleep(1);
      distance = sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT);
      actual_velocity_right = motors_get_motor_speed(ROBOT_WHEEL_RIGHT_PORT);
      actual_velocity_left = motors_get_motor_speed(ROBOT_WHEEL_LEFT_PORT);
      currenttime = time(NULL);
      elapsed = static_cast<int>(difftime(currenttime, starttime));
      printf("move_until_collision dist=%d, velocity=%d, left=%d, right=%d, elapsed=%d\n", distance, velocity, actual_velocity_left, actual_velocity_right, elapsed);
    } while ((elapsed < 30) and /*((distance<120) or (distance>180)) and*/ ((abs(actual_velocity_left)>0) or (abs(actual_velocity_right)>0)));
    motors_stop(ROBOT_WHEEL_RIGHT_PORT);
    motors_stop(ROBOT_WHEEL_LEFT_PORT);
    sleep(1);
    printf("move_until_collision done\n");
  }
public:
  void move_until_target()
  {
    printf("move_until_target start\n");
    int distance = 0;
    bool reached_target = false;
    while (not reached_target)
    {
      move_until_collision();
      distance = sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT);
      if ((distance > 120) and (distance < 180))
      {
        reached_target = true;
      }
      else
      {
        motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, 60);
        motors_set_speed(ROBOT_WHEEL_LEFT_PORT, 60);
        sleep(2);
        motors_stop(ROBOT_WHEEL_RIGHT_PORT);
        motors_stop(ROBOT_WHEEL_LEFT_PORT);
        sleep(1);
        command_turn_angle(-135);
      }
    }
    printf("move_until_target done\n");
  }

  void move_and_turn(int distance_mm, int degrees)
  {
    printf("move_and_turn start (distance=%d, angle=%d)\n", distance_mm, degrees);
    // Drive backward (positive velocity is moving in direction of shooter)
    if (distance_mm != 0)
    {
      int velocity = (distance_mm>0) ? -60 : 60;
      motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, velocity);
      motors_set_speed(ROBOT_WHEEL_LEFT_PORT, velocity);
      sleep_ms((abs(distance_mm) * 1000)/ 60);
      motors_stop(ROBOT_WHEEL_RIGHT_PORT);
      motors_stop(ROBOT_WHEEL_LEFT_PORT);
      sleep(1);
    }
    if (degrees != 0)
    {
      command_turn_angle(degrees);
    }
    printf("move_and_turn done (distance=%d, angle=%d)\n", distance_mm, degrees);
  }
};

static CBallHandler* ballhandler = NULL;
static CWheelHandler* wheelhandler = NULL;

void ball_initialize()
{
  fastmath_init();
  ballhandler = new CBallHandler();
  wheelhandler = new CWheelHandler();
}


void ball_terminate()
{
  delete wheelhandler;
  delete ballhandler;
  motors_stop(ROBOT_WHEEL_RIGHT_PORT);
  motors_stop(ROBOT_WHEEL_LEFT_PORT);
  motors_stop(ROBOT_BALL_THROW_PORT);
  motors_stop(ROBOT_BALL_CATCH_PORT);
}


void ball_execute(bool* keep_running)
{
  printf("ball execute start\n");
  COLOR_CODE color;
  
  /*
  ballhandler->open();
  sleep(5);
  printf("\n\nHALF");
  ballhandler->half_down();
  sleep(5);
  printf("\n\nDOWN");
  ballhandler->down();
  sleep(5);
  printf("\n\nHALF");
  ballhandler->half_down();
  sleep(5);
  printf("\n\nUP");
  ballhandler->up();
  sleep(5);
  printf("\n\nHALF");
  ballhandler->half_down();
  sleep(5);
  printf("\n\nUP");
  ballhandler->up();
  sleep(5);
  printf("\n\nDOWN");
  ballhandler->down();
  sleep(5);
  ballhandler->close();
  sleep(5);
  */
  
  while(1)
  {
    wheelhandler->move_until_target();
    wheelhandler->move_and_turn(-200, 180);
    ballhandler->open();
    ballhandler->half_down();
    wheelhandler->move_and_turn(-200, 0);
    
    if (ballhandler->trycatch(color))
    {
      int angle;
      switch (color)
      {
        case COLOR_CODE_GREEN:
          angle = 180;
          break;
        case COLOR_CODE_YELLOW:
          angle = 170;
          break;
        case COLOR_CODE_RED:
          angle = 160;
          break;
        case COLOR_CODE_BLUE:
          angle = 150;
          break;
        default:
          angle = 180;
      }
      if (ballhandler->open())
      {
        // Raise bar a bit, otherwise get stuck when driving backward
        ballhandler->half_down();
      }
      // Drive a little bit backward, and turn
      wheelhandler->move_and_turn(200, angle);
      // Shoot
      if (ballhandler->open())
      {
        ballhandler->shoot();
      }
      ballhandler->down();
      ballhandler->close();
      // turn with sensors along target
      wheelhandler->move_and_turn(0, 90 - angle);
    }
    else // No ball
    {
      if (ballhandler->open())
      {
        // Raise bar a bit, otherwise get stuck when driving backward
        ballhandler->half_down();
      }
      // Drive a little bit backward, and turn, with sensors along target
      wheelhandler->move_and_turn(200, 90);
    }
  
    // Position robot with sensors pointing to target
    wheelhandler->move_and_turn(200, 90);
  }
  
  printf("ball execute done\n");
  return;
}
