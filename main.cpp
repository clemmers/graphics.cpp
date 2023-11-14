#include "graphics.h"
#include <vector>
#include <array>


/* EXAMPLE SCENE CODE */
int main()
{
    std::vector<std::array<long double, 3>> squareVerts {
            {25.0L, 25.0L, 25.0L},
            {25.0L, -25.0L, 25.0L},
            {-25.0L, -25.0L, 25.0L},
            {-25.0L, 25.0L, 25.0L}
    };


    Color RED = Color(255,0,0);
    Polyhedron test(200.0L, 200.0L, 200.0L, squareVerts, RED, Color(), std::vector<std::array<long double, 3>>{{0,0,1}});
    Cube cube(400, 300, 100, 100);
    RegularPolygon hexagon(275, 300, 100, 8, 50, Color(0, 255, 255));

    std::vector<Polyhedron*> objects{&test, &hexagon, &cube};
    GraphicsWindow window(800, 400, &objects, Color(200,200,200), Camera(0,0,-1000,0,0,0));
    
    window.open();
    int direction = 1;
    long double speed = 0.001;
    while(window.isOpen()) {
        test.translateYaw(0.01);
        speed *= 1.0001;
        hexagon.translatePitch(-0.005);
        cube.translateAngle(0.001,0.001,0.001);
        if(cube.getX() > 600 || cube.getX() < 200) direction *= -1;
        cube.translate(direction * 0.1, 0, 0);
        window.refresh();
    }
}
