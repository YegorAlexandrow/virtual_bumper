#ifndef BUMPER_H_INCLUDED
#define BUMPER_H_INCLUDED

#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define PI      3.14159265358979323846264338327950288419716939937510582097
#define SIN30   0.5
#define COS150 -0.86602540378443864676372317075293618347140262690519031402
#define COS30   0.86602540378443864676372317075293618347140262690519031402

#define N_OF_WALLS 6

#define WALL_FRONT      200.
#define WALL_SIDE_SHORT 85.
#define WALL_SIDE_LONG  180.
#define WALL_BACK       100.

#define N_OF_VLX        18

/** Константы, требующие фактической подстройки **/

/* Расстояния бампера по умолчанию */
#define BUMPER_DEFAULT_FLOOR    30.
#define BUMPER_DEFAULT_CEIL     80.

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

#define COS_STOP -0.5

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

Vect newVect(double _x, double _y);
VlxPoint newVlxPoint(double _x, double _y, Vect _unit, double _wt, int _wall_index);
void makeBodyPoints();
void makeVlxPoints();
Vect rv(Vect v, double angle);
double react(double x, double v, double floor, double ceil);
double reactAngular(VlxPoint vlx, double offset, double delta);


/**
* Функция корректировки скоростей с учётом виртульного бампера
* @param Vx - компонента вектора скорости по оси X в системе координат робота (в мм)
* @param Vy - компонента вектора скорости по оси Y в системе координат робота (в мм)
* @param Vz - скорость вращения вокруг своей оси в системе координат робота (в градусах)
* @param dists - массив расстояний с VLX-ов (в мм)
* @return Скорректированный вектор скорости (Vx, Vy, Vz)
*/
Velocity bump(double Vx, double Vy, double Vz, uint16_t* dists);

/**
* Устанавливает значение границ виртуального бампера для i-ой стены
* @param i - индекс стены (нумерацию см. на рис.)
* @param ceil - новое значение верхней границы
* @param floor - новое значение нижней границы
*/
void setBumperValueAt(int i, double floor, double ceil);


/**
* Устанавливает значение верхней границы виртуального бампера для i-ой стены
* @param i - индекс стены (нумерацию см. на рис.)
* @param value - новое значение верхней границы
*/
void setBumperCeilAt(int i, double value);

/**BUMPER_DEFAULT_FLOOR
* Устанавливает значение нижней границы виртуального бампера для i-ой стены
* @param i - индекс стены (нумерацию см. на рис.)
* @param value - новое значение нижней границы
*/
void setBumperFloorAt(int i, double value);


/**/void initVirtualBumper();



#endif
