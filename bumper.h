#ifndef BUMPER_H_INCLUDED
#define BUMPER_H_INCLUDED

#include <stdlib.h>
#include <math.h>

#define PI      3.14159265358979323846264338327950288419716939937510582097
#define SIN30   0.5
#define COS150 -0.86602540378443864676372317075293618347140262690519031402
#define COS30   0.86602540378443864676372317075293618347140262690519031402

#define N_OF_WALLS 6

#define WALL_FRONT      200.
#define WALL_SIDE_SHORT 85.
#define WALL_SIDE_LONG  180.
#define WALL_BACK       100.

/** Константы, требующие фактической подстройки **/

/* Расстояния бампера по умолчанию */
#define BUMPER_DEFAULT_FLOOR    25.
#define BUMPER_DEFAULT_CEIL     70.

/*Расстояния бампера для стороны с выкинутым харвестером */
#define BUMPER_HARVESTER_FLOOR  80.
#define BUMPER_HARVESTER_CEIL   120.

/* Расстояния бампера для стороны с золотой шайбой */
#define BUMPER_GOLDENIUM_FLOOR  50.
#define BUMPER_GOLDENIUM_CEIL   90.

/* Расстояния бампера для стороны с золотой шайбой */
#define BUMPER_CONTACT_FLOOR    -5.
#define BUMPER_CONTACT_CEIL     2.

#define VLX_TAB 8.96                /* Расстояние от центра vlx'a до центра платы */
#define VLX_ANGLE 18. / 180 * PI    /* Угол наклона vlx'ов относительно нормали поверхнисти (>0 - vlx'ы смотрят наружу) */

/*************************************************/

typedef struct {
    double x;
    double y;
} Vect;

typedef struct {
    double vx;
    double vy;
    double vz;
} Velocity;

typedef struct {
    Vect pt;
    Vect unit;
    double wt;
    int wall_index;
} VlxPoint;

VlxPoint* VLXes;

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

/**/Vect newVect(double _x, double _y);
/**/VlxPoint newVlxPoint(double _x, double _y, Vect _unit, double _wt, int _wall_index);
/**/Vect* makeBodyPoints();
/**/Vect makeNormale(double dx, double dy);
/**/Vect* makeNormales(Vect* body);
/**/VlxPoint* makeVlxPoints(Vect* body);
/**/Vect rv(Vect v, double angle);
/**/double react(double x, double v, double floor, double ceil);
/**/double reactAngular(VlxPoint vlx, double offset, double delta);
/**/Velocity bump(double Vx, double Vy, double Vz, double* dists);
/**/void setBumperCeilAt(int i, double value);
/**/void setBumperFloorAt(int i, double value);
/**/void initVirtualBumper();


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

Vect* makeBodyPoints()
{
    Vect* body = (Vect*) malloc(N_OF_WALLS * sizeof(Vect));

    double y_b = WALL_FRONT/2 + WALL_SIDE_SHORT*SIN30;
    double x_b = WALL_SIDE_SHORT*COS150;
    double k, tab;
    int i;
    Vect m;

    body[0] = newVect(0, WALL_FRONT/2);

    body[1] = newVect(x_b, y_b);
    body[2] = newVect(x_b - sqrt(pow(WALL_SIDE_LONG, 2) - pow(pow(2*y_b-WALL_BACK, 2)/(2*(2*y_b-WALL_BACK)), 2)), WALL_BACK/2);

    for(i=0; i<3; i++) body[i+3] = newVect(body[2-i].x, -body[2-i].y);

    k = -(body[2].x-body[1].x)/(body[2].y-body[1].y);
    m = newVect((body[5].x+body[4].x)/2, (body[5].y+body[4].y)/2);
    tab = m.x - m.y/k;

    for(i=0; i<6; i++) body[i].x -= tab;

    return body;
}

Vect makeNormale(double dx, double dy)
{
    double l = sqrt(dx*dx + dy*dy);
    return newVect(dy/l, -dx/l);
}

Vect* makeNormales(Vect* body)
{
    Vect* norms = (Vect*) malloc(N_OF_WALLS * sizeof(Vect));
    int i;

    for(i=0; i<N_OF_WALLS; i++) {
        norms[i] = makeNormale(body[(i+1)%N_OF_WALLS].x - body[i].x, body[(i+1)%N_OF_WALLS].y - body[i].y);
    }

    return norms;
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

VlxPoint* makeVlxPoints(Vect* body)
{
    VlxPoint* VLXes = (Vect*) malloc((3*2 + 3*4) * sizeof(VlxPoint));

    //front side
    addVlxPair(VLXes+0, body[5], body[0], WALL_FRONT-20.69, 0.25, 0);
    addVlxPair(VLXes+16, body[5], body[0], 20.69, 0.25, 0);

    //short side
    addVlxPair(VLXes+2, body[0], body[1], WALL_SIDE_SHORT/2, 0.5, 1);

    //long side
    addVlxPair(VLXes+4, body[1], body[2], 45, 0.25, 2);
    addVlxPair(VLXes+6, body[1], body[2], WALL_SIDE_LONG-23, 0.25, 2);

    //back side
    addVlxPair(VLXes+8, body[2], body[3], WALL_BACK/2, 0.5, 3);

    //long side
    addVlxPair(VLXes+10, body[3], body[4], 45, 0.25, 4);
    addVlxPair(VLXes+12, body[3], body[4], WALL_SIDE_LONG-23, 0.25, 4);

    //short side
    addVlxPair(VLXes+14, body[4], body[5], WALL_SIDE_SHORT/2, 0.5, 5);

    return VLXes;
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
    return (alpha2 - alpha1) / PI * 180;
}

char sign(double x)
{
    return (x > 0) - (x < 0);
}

double hypot(double x, double y) {
    return sqrt(x*x + y*y);
}

/**
* Функция корректировки скоростей с учётом виртульного бампера
* @param Vx - компонента вектора скорости по оси X в системе координат робота (в мм)
* @param Vy - компонента вектора скорости по оси Y в системе координат робота (в мм)
* @param Vz - скорость вращения вокруг своей оси в системе координат робота (в градусах)
* @param dists - массив расстояний с VLX-ов (в мм)
* @return Скорректированный вектор скорости (Vx, Vy, Vz)
*/
Velocity bump(double Vx, double Vy, double Vz, double* dists)
{
    Velocity v_old = newVelocity(Vx, Vy, Vz);

    char signs[] = {sign(Vx), sign(Vy), sign(Vz)};
    int i;
    double v_proj;
    double c = fabs(cos(VLX_ANGLE));
    double v_angle = atan2(Vy, Vx);
    double v_abs = hypot(Vx, Vy);
    double r;

    for(i=0; i<18; i++)
    {
        double FLOOR = floors[VLXes[i].wall_index];
        double CEIL  = ceils[VLXes[i].wall_index];

        v_proj = v_abs * fabs(cos(v_angle - atan2(VLXes[i].unit.y, VLXes[i].unit.x)));
        r = react(dists[i]/c, v_proj/VLXes[i].wt, FLOOR/c, CEIL/c);

        Vx += VLXes[i].unit.x * r * VLXes[i].wt;
        Vy += VLXes[i].unit.y * r * VLXes[i].wt;
        Vz += dists[i]<CEIL ? reactAngular(VLXes[i], CEIL, dists[i]-CEIL) : 0;
    }

    if(sign(Vx) != signs[0]) Vx = 0;
    if(sign(Vy) != signs[1]) Vy = 0;

    return newVelocity(Vx, Vy, Vz);
}

/**
* Устанавливает значение верхней границы виртуального бампера для i-ой стены
* @param i - индекс стены (нумерацию см. на рис.)
* @param value - новое значение верхней границы
*/
void setBumperCeilAt(int i, double value) { ceils[i] = value; }

/**BUMPER_DEFAULT_FLOOR
* Устанавливает значение нижней границы виртуального бампера для i-ой стены
* @param i - индекс стены (нумерацию см. на рис.)
* @param value - новое значение нижней границы
*/
void setBumperFloorAt(int i, double value) { floors[i] = value; }

/**
* Устанавливает значение границ виртуального бампера для i-ой стены
* @param i - индекс стены (нумерацию см. на рис.)
* @param ceil - новое значение верхней границы
* @param floor - новое значение нижней границы
*/
void setBumperValueAt(int i, double floor, double ceil) { floors[i] = floor; ceils[i] = ceil; }

/**
* Инициализация виртуального бампера.
* @note Должна быть вызвана до первого вызова bump()
*/
void initVirtualBumper() { VLXes = makeVlxPoints(makeBodyPoints()); }

#endif
