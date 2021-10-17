//
// Created by stew on 23/07/21.
//

#ifndef APPLICATIONS_HELPER_FUNCTIONS_HPP
#define APPLICATIONS_HELPER_FUNCTIONS_HPP




struct Point {
  double x, y;
};

struct line {
  Point p1, p2;
};



// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the floats i_x and i_y.
bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
                           float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
  float s1_x, s1_y, s2_x, s2_y;
  s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
  s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

  float s, t;
  s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
  t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
  {
    // Collision detected
    if (i_x != NULL)
      *i_x = p0_x + (t * s1_x);
    if (i_y != NULL)
      *i_y = p0_y + (t * s1_y);
    return true;
  }

  return false; // No collision
}


bool onLine(line l1, Point p) {   //check whether p is on the line or not
  if(p.x <= std::max(l1.p1.x, l1.p2.x) && p.x <= std::min(l1.p1.x, l1.p2.x) &&
     (p.y <= std::max(l1.p1.y, l1.p2.y) && p.y <= std::min(l1.p1.y, l1.p2.y)))
    return true;

  return false;
}

int direction(Point a, Point b, Point c) {
  int val = (b.y-a.y)*(c.x-b.x)-(b.x-a.x)*(c.y-b.y);
  if (val == 0)
    return 0;     //colinear
  else if(val < 0)
    return 2;    //anti-clockwise direction
  return 1;    //clockwise direction
}

bool isIntersect(line l1, line l2) {
  //four direction for two lines and points of other line
  int dir1 = direction(l1.p1, l1.p2, l2.p1);
  int dir2 = direction(l1.p1, l1.p2, l2.p2);
  int dir3 = direction(l2.p1, l2.p2, l1.p1);
  int dir4 = direction(l2.p1, l2.p2, l1.p2);

  if(dir1 != dir2 && dir3 != dir4)
    return true; //they are intersecting

  if(dir1==0 && onLine(l1, l2.p1)) //when p2 of line2 are on the line1
    return true;

  if(dir2==0 && onLine(l1, l2.p2)) //when p1 of line2 are on the line1
    return true;

  if(dir3==0 && onLine(l2, l1.p1)) //when p2 of line1 are on the line2
    return true;

  if(dir4==0 && onLine(l2, l1.p2)) //when p1 of line1 are on the line2
    return true;

  return false;
}

#endif //APPLICATIONS_HELPER_FUNCTIONS_HPP
