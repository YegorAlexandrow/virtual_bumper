#include "bumper.h"

VlxPoint VLXes[N_OF_VLX];
Vect bumper_body[N_OF_WALLS];

double ceils[6]  =
    {
    BUMPER_DEFAULT_CEIL,
    BUMPER_DEFAULT_CEIL,
    BUMPER_DEFAULT_CEIL,
    BUMPER_DEFAULT_CEIL,
    BUMPER_DEFAULT_CEIL,
    BUMPER_DEFAULT_CEIL
    };

double floors[6] =
    {
    BUMPER_DEFAULT_FLOOR,
    BUMPER_DEFAULT_FLOOR,
    BUMPER_DEFAULT_FLOOR,
    BUMPER_DEFAULT_FLOOR,
    BUMPER_DEFAULT_FLOOR,
    BUMPER_DEFAULT_FLOOR
    };

Vect newVect(double _x, double _y)
{
    Vect v;
    v.x = _x;
    v.y = _y;

    return v;
}

Velocity newVelocity(double _vx, double _vy, double _vz)
{
    Velocity v;
    v.vx = _vx;
    v.vy = _vy;
    v.vz = _vz;

    return v;
}

VlxPoint newVlxPoint(double _x, double _y, Vect _unit, double _wt, int _wall_index)
{
    VlxPoint p;
    p.pt   = newVect(_x, _y);
    p.unit = _unit;
    p.wt   = _wt;
    p.wall_index = _wall_index;

    return p;
}

void makeBodyPoints()
{
    double y_b = WALL_FRONT/2 + WALL_SIDE_SHORT*SIN30;
    double x_b = WALL_SIDE_SHORT*COS150;
    double k, tab;
    int i;
    Vect m;

    bumper_body[0] = newVect(0, WALL_FRONT/2);

    bumper_body[1] = newVect(x_b, y_b);
    bumper_body[2] = newVect(x_b - sqrt(pow(WALL_SIDE_LONG, 2) - pow(pow(2*y_b-WALL_BACK, 2)/(2*(2*y_b-WALL_BACK)), 2)), WALL_BACK/2);

    for(i=0; i<3; i++) bumper_body[i+3] = newVect(bumper_body[2-i].x, -bumper_body[2-i].y);

    k = -(bumper_body[2].x-bumper_body[1].x)/(bumper_body[2].y-bumper_body[1].y);
    m = newVect((bumper_body[5].x+bumper_body[4].x)/2, (bumper_body[5].y+bumper_body[4].y)/2);
    tab = m.x - m.y/k;

    for(i=0; i<6; i++) bumper_body[i].x -= tab;
}

Vect rv(Vect v, double angle)
{
    return newVect(v.x*cos(angle) - v.y*sin(angle), v.x*sin(angle) + v.y*cos(angle));
}

void addVlxPair(VlxPoint* curr, Vect wall_start, Vect wall_end, double offset, double wt, int wall_index)
{
    double dx = wall_end.x - wall_start.x;
    double dy = wall_end.y - wall_start.y;
    double l = sqrt(dx*dx + dy*dy);

    Vect unit = newVect(dx/l, dy/l);
    Vect norm = newVect(unit.y, -unit.x);

    curr[0] = newVlxPoint(wall_start.x + unit.x*(offset-VLX_TAB), wall_start.y + unit.y*(offset-VLX_TAB), rv(norm, -VLX_ANGLE), wt, wall_index);
    curr[1] = newVlxPoint(wall_start.x + unit.x*(offset+VLX_TAB), wall_start.y + unit.y*(offset+VLX_TAB), rv(norm, +VLX_ANGLE), wt, wall_index);
}

void makeVlxPoints()
{
    //front side
    addVlxPair(VLXes+0, bumper_body[5], bumper_body[0], WALL_FRONT-20.69, 0.25, 0);
    addVlxPair(VLXes+16, bumper_body[5], bumper_body[0], 20.69, 0.25, 0);

    //short side
    addVlxPair(VLXes+2, bumper_body[0], bumper_body[1], WALL_SIDE_SHORT/2, 0.5, 1);

    //long side
    addVlxPair(VLXes+4, bumper_body[1], bumper_body[2], 45, 0.25, 2);
    addVlxPair(VLXes+6, bumper_body[1], bumper_body[2], WALL_SIDE_LONG-23, 0.25, 2);

    //back side
    addVlxPair(VLXes+8, bumper_body[2], bumper_body[3], WALL_BACK/2, 0.5, 3);

    //long side
    addVlxPair(VLXes+10, bumper_body[3], bumper_body[4], 45, 0.25, 4);
    addVlxPair(VLXes+12, bumper_body[3], bumper_body[4], WALL_SIDE_LONG-23, 0.25, 4);

    //short side
    addVlxPair(VLXes+14, bumper_body[4], bumper_body[5], WALL_SIDE_SHORT/2, 0.5, 5);
}

double react(double x, double v, double floor, double ceil)
{
    double a, b, c;

    if(x > ceil) return 0;

    if(x > floor)
    {
        v *= -1;
        a = v / pow(floor - ceil, 2);
        b = -(2*ceil*v) / pow(ceil-floor, 2);
        c = (v*ceil*ceil) / pow(ceil-floor, 2);

        return a*x*x + b*x + c;
    }

    return -v;
}

double reactAngular(VlxPoint vlx, double offset, double delta)
{
    double alpha1 = atan2(vlx.pt.y + vlx.unit.y*offset, vlx.pt.x + vlx.unit.x*offset);
    double alpha2 = atan2(vlx.pt.y + vlx.unit.y*(offset+delta), vlx.pt.x + vlx.unit.x*(offset+delta));
    return (alpha2 - alpha1)/* / PI * 180*/;
}

char sign(double x)
{
    return (x > 0) - (x < 0);
}

double hypot(double x, double y) {
    return sqrt(x*x + y*y);
}

/**
* ������� ������������� ��������� � ������ ����������� �������
* @param Vx - ���������� ������� �������� �� ��� X � ������� ��������� ������ (� ��)
* @param Vy - ���������� ������� �������� �� ��� Y � ������� ��������� ������ (� ��)
* @param Vz - �������� �������� ������ ����� ��� � ������� ��������� ������ (� ��������)
* @param dists - ������ ���������� � VLX-�� (� ��)
* @return ����������������� ������ �������� (Vx, Vy, Vz)
*/
Velocity bump(double Vx, double Vy, double Vz, uint16_t* dists)
{
    Velocity v_old = newVelocity(Vx, Vy, Vz);

    char signs[] = {sign(Vx), sign(Vy), sign(Vz)};
    int i;
    double v_proj;
    double c = fabs(cos(VLX_ANGLE));
    double v_angle = atan2(Vy, Vx);
    double v_abs = hypot(Vx, Vy);
    double r;
    double abs_v_new, abs_v_old;

    for(i=0; i<18; i++)
    {
        double FLOOR = floors[VLXes[i].wall_index];
        double CEIL  = ceils[VLXes[i].wall_index];

        if(dists[i] != 0)
        {
            v_proj = v_abs * fabs(cos(v_angle - atan2(VLXes[i].unit.y, VLXes[i].unit.x)));
            r = react((double)dists[i]/c, v_proj/VLXes[i].wt, FLOOR/c, CEIL/c);

            Vx += VLXes[i].unit.x * r * VLXes[i].wt;
            Vy += VLXes[i].unit.y * r * VLXes[i].wt;
            //Vz += dists[i]<CEIL ? reactAngular(VLXes[i], CEIL, dists[i]-CEIL) : 0;
        }
    }

    //if(sign(Vx) != signs[0]) Vx = 0;
    //if(sign(Vy) != signs[1]) Vy = 0;

    abs_v_new = sqrt(Vx*Vx + Vy*Vy);
    abs_v_old = sqrt(v_old.vx * v_old.vx + v_old.vy * v_old.vy);

    if((Vx*v_old.vx + Vy*v_old.vy)/(abs_v_new*abs_v_old) < COS_STOP) Vx = Vy = Vz = 0;

    if(abs_v_new - abs_v_old >= 0.001)
    {
        double k = abs_v_new / abs_v_old;
        Vx *= k;
        Vy *= k;
    }

    return newVelocity(Vx, Vy, Vz);
}

/**
* ������������� �������� ������� ������� ������������ ������� ��� i-�� �����
* @param i - ������ ����� (��������� ��. �� ���.)
* @param value - ����� �������� ������� �������
*/
void setBumperCeilAt(int i, double value) { ceils[i] = value; }

/**BUMPER_DEFAULT_FLOOR
* ������������� �������� ������ ������� ������������ ������� ��� i-�� �����
* @param i - ������ ����� (��������� ��. �� ���.)
* @param value - ����� �������� ������ �������
*/
void setBumperFloorAt(int i, double value) { floors[i] = value; }

/**
* ������������� �������� ������ ������������ ������� ��� i-�� �����
* @param i - ������ ����� (��������� ��. �� ���.)
* @param ceil - ����� �������� ������� �������
* @param floor - ����� �������� ������ �������
*/
void setBumperValueAt(int i, double floor, double ceil) { floors[i] = floor; ceils[i] = ceil; }

/**
* ������������� ������������ �������.
* @note ������ ���� ������� �� ������� ������ bump()
*/
void initVirtualBumper() { makeBodyPoints(); makeVlxPoints(); }

