#include <stdio.h>
#include "bumper.h"

int main()
{
    int i;
    double dists[] = {65, 65, 51, 51, 60, 60, 60, 60, 100, 100, 60, 60, 80, 80, 85, 50, 50, 60};
    double vx = 700, vy = -210, vz = 0;
    Velocity v;

    initVirtualBumper();

    v = bump(vx, vy, vz, dists);
    printf("(%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)\n", vx, vy, vz, v.vx, v.vy, v.vz);

    setBumperCeilAt(0, BUMPER_HARVESTER_CEIL);
    setBumperFloorAt(0, BUMPER_HARVESTER_FLOOR);

    v = bump(vx, vy, vz, dists);
    printf("(%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)\n", vx, vy, vz, v.vx, v.vy, v.vz);


    vx = -200, vy = 50, vz = 0;
    setBumperValueAt(0, BUMPER_DEFAULT_FLOOR, BUMPER_DEFAULT_FLOOR);
    setBumperValueAt(3, BUMPER_CONTACT_FLOOR, BUMPER_CONTACT_CEIL);
    v = bump(vx, vy, vz, dists);
    printf("(%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)\n", vx, vy, vz, v.vx, v.vy, v.vz);

    return 0;
}
