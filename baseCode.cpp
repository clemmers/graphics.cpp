// g++ -o baseCode baseCode.cpp -lSDL2 -lGL -lGLU -lglut -I/usr/include/SDL2 -L/usr/lib

#include <SDL2/SDL.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <array>
#include <chrono>

#define PI 3.141592653589793238462643383279502884L
#define LEFT -1
#define UP -1
#define RIGHT 1
#define DOWN 1


const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

class GraphicsWindow;
class Polygon;

class Color {
private:
    int red;
    int green;
    int blue;

public:
    Color() {
		red = 0;
		green = 0;
		blue = 0;
	}

    Color(Uint8 red, Uint8 green, Uint8 blue) {
		this->red = red;
		this->green = green;
		this->blue = blue;
	}

    int getRed() const {
        return red;
    }

    int getGreen() const {
        return green;
    }

    int getBlue() const {
        return blue;
    }

    void setRed(int red) {
        this->red = red;
    }

    void setGreen(int green) {
        this->green = green;
    }

    void setBlue(int blue) {
        this->blue = blue;
    }
};

std::vector<std::array<long double, 3>> calculateRegularPolygonVertices (int numSides, long double circumradius) {
    std::vector<std::array<long double, 3>> vertices {};
        long double angle = 2 * PI / numSides;
        for (int i = 0; i < numSides; i++)
        {
            vertices.push_back(std::array<long double, 3>{circumradius * cos(i * angle), circumradius * sin(i * angle), 0});
        }
        return vertices;
}

class GraphicalObject {
protected:
	long double x;
	long double y;
    long double z;
    long double roll;
    long double pitch;
    long double yaw;

public:

    //virtual void draw(GraphicsWindow* graphics) = 0;
    //virtual std::vector<std::array<long double, 3>>* getVertices() = 0;
    GraphicalObject(long double x = 0, long double y = 0, long double z = 0, long double roll = 0, long double pitch = 0, long double yaw = 0)
	: x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}

    long double getX() const {
        return x;
    }
    
    long double getY() const {
        return y;
    }

    long double getZ() const {
        return z;
    }

    long double getYaw() const {
        return yaw;
    }
    
    long double getPitch() const {
        return pitch;
    }

    long double getRoll() const {
        return roll;
    }

    void setX(long double x) {
        this->x = x;
    }
    
    void setY(long double y) {
        this->y = y;
    }

    void setZ(long double z) {
        this->z = z;
    }

    void setYaw(long double yaw) {
        this->yaw = yaw;
    }
    
    void setPitch(long double pitch) {
        this->pitch = pitch;
    }

    void setRoll(long double roll) {
        this->roll = roll;
    }

    void translateRoll(long double dRoll) {
        //std::cout << roll << std::endl;
        roll += dRoll;
        // new slang when you notice the stripes!
    }

    void translatePitch(long double dPitch) {
        pitch += dPitch;
    }

    void translateYaw(long double dYaw) {
        yaw += dYaw;
    }

    void translate(long double dx, long double dy, long double dz) {
        x += dx;
        y += dy;
        z += dz;
    }
};

class Camera : public GraphicalObject {
private:
    long double vX;
    long double vY;
    long double vZ;
    long double friction;
    long double movementSpeed;
    long double lookSpeed;
    std::chrono::time_point<std::chrono::system_clock> timeWhenLastUpdated;

public:
    Camera(long double x = 0, long double y = 0, long double z = 0,
           long double roll = 0, long double pitch = 0, long double yaw = 0,
           long double movementSpeed = 1, long double lookSpeed = 1.0L/(2*PI),
           long double friction = 0.5, long double vX = 0, long double vY = 0,
           long double vZ = 0)
        : GraphicalObject(x,y,z,roll,pitch,yaw), friction(friction),
          movementSpeed(movementSpeed), lookSpeed(lookSpeed),
          vX(vX), vY(vY), vZ(vZ),
          timeWhenLastUpdated(std::chrono::high_resolution_clock::now()) {}

    void update() {
        long double timeSinceLastFrame = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now() - timeWhenLastUpdated).count() / 100000000.0L;
        x += vX * timeSinceLastFrame;
        y += vY * timeSinceLastFrame;
        z += vZ * timeSinceLastFrame;
        vX *= pow(friction, timeSinceLastFrame);
        vY *= pow(friction, timeSinceLastFrame);
        vZ *= pow(friction, timeSinceLastFrame);
        timeWhenLastUpdated = std::chrono::high_resolution_clock::now();
    }

    void translateVX(int direction) {
        vX += direction * movementSpeed;
    }
    
    void translateVY(int direction) {
        vY += direction * movementSpeed;
    }

    void translateVZ(int direction) {
        vZ += direction * movementSpeed;
    }

    void translateLook(int x, int y) {
        translateYaw(x * lookSpeed);
        translatePitch(y * lookSpeed);
    }

};

std::array<long double, 3> rotateX(long double x, long double y, long double z, long double angle) {
    std::array<long double, 3> rotatedVector {
        x,
        y * cos(angle) + z * -sin(angle),
        y * sin(angle) + z * cos(angle)
    };

    return rotatedVector;
}

std::array<long double, 3> rotateY(long double x, long double y, long double z, long double angle) {
    std::array<long double, 3> rotatedVector {
        x * cos(angle) + z * sin(angle),
        y,
        x * -sin(angle) + z * cos(angle)
    };

    return rotatedVector;
}

std::array<long double, 3> rotateZ(long double x, long double y, long double z, long double angle) {
    std::array<long double, 3> rotatedVector {
        x * cos(angle) + y * -sin(angle),
        x * sin(angle) + y * cos(angle),
        z
    };

    return rotatedVector;
}


class Polygon : public GraphicalObject {
protected:
	std::vector<std::array<long double, 3>> vertices;
    Color color;

public:
	Polygon(long double x = 0, long double y = 0, long double z = 0, std::vector<std::array<long double, 3>> vertices = {}, Color color = Color(), long double roll = 0, long double pitch = 0, long double yaw = 0)
    : GraphicalObject(x, y, z, roll, pitch, yaw), vertices(vertices), color(color) {}


    void draw(GraphicsWindow& graphics);


    Color& getColor() {
        return color;
    }

    std::vector<std::array<long double, 3>>& getVertices() {
        return vertices;
    }
};


class GraphicsWindow {
private:
    int screenWidth;
    int screenHeight;
    std::vector<Polygon*>* objects;
    Color backgroundColor;
    SDL_Window* window;
    SDL_Renderer* renderer;
    Uint32 prevTicks;
    bool quit;
    SDL_Event e;
    int frameCount;
    std::string windowTitle;
    Camera camera;
    int prevMouseX;
    int prevMouseY;

public:
    GraphicsWindow(int screenWidth, int screenHeight, std::vector<Polygon*>* objects, Color backgroundColor, Camera camera)
    : screenWidth(screenWidth), screenHeight(screenHeight), objects(objects), backgroundColor(backgroundColor), camera(camera), prevMouseX(0), prevMouseY(0) {}


    void drawLine(long double x1, long double y1, long double z1, long double x2, long double y2, long double z2, Color& color) {
        SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
        SDL_RenderDrawLine(renderer, (int) (x1 - camera.getX()), (int) (y1 - camera.getY()), (int) (x2 - camera.getX()), (int) (y2 - camera.getY()));
        //SDL_RenderDrawLine(renderer, (int) ((x1 - camera.getX())/(z1 - camera.getZ())), (int) ((y1 - camera.getY())/(z1 - camera.getZ())), (int) ((x2 - camera.getX())/(z2 - camera.getZ())), (int) ((y2 - camera.getY())/(z2 - camera.getZ())));
    }

    void drawObject(Polygon& object) {
        std::vector<std::array<long double, 3>> rotatedVertices{};
        std::vector<std::array<long double, 3>>& verticesPointer = object.getVertices();
        for(int i = 0; i < verticesPointer.size(); i++) {
            std::array<long double, 3> tempVerts = rotateX(verticesPointer[i][0],verticesPointer[i][1],verticesPointer[i][2],object.getRoll());
            tempVerts = rotateY(tempVerts[0],tempVerts[1],tempVerts[2],object.getPitch());
            tempVerts = rotateZ(tempVerts[0],tempVerts[1],tempVerts[2],object.getYaw());
            rotatedVertices.push_back(tempVerts);
        }
        for(int i = 0; i < rotatedVertices.size(); i++) {
            for(int j = 0; j < rotatedVertices.size(); j++) {
                this->drawLine(rotatedVertices[i][0] + object.getX(), rotatedVertices[i][1] + object.getY(), rotatedVertices[i][2] + object.getZ(), rotatedVertices[j][0] + object.getX(), rotatedVertices[j][1] + object.getY(), rotatedVertices[j][2] + object.getZ(), object.getColor());
            }
        }
    }

    bool isOpen() {
        return !quit;
    }

    int open() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            return 1;
        }

        window = SDL_CreateWindow("c++ graphics - fps:", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, screenWidth, screenHeight, SDL_WINDOW_SHOWN);
        if (!window)
        {
            std::cerr << "SDL window creation failed: " << SDL_GetError() << std::endl;
            return 1;
        }

        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer)
        {
            std::cerr << "SDL renderer creation failed: " << SDL_GetError() << std::endl;
            return 1;
        }

        
        quit = false;
        
        prevTicks = SDL_GetTicks();
        frameCount = 0;
        windowTitle = "fps";

        return 0;
    }

    int refresh() {
        if (quit) {
            this->destroy();
            return 1;
        }
        SDL_SetRenderDrawColor(renderer, backgroundColor.getRed(), backgroundColor.getGreen(), backgroundColor.getBlue(), 255);
        SDL_RenderClear(renderer);
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
            if (e.type == SDL_MOUSEMOTION) {
                camera.translateLook(prevMouseX - e.motion.x, prevMouseY - e.motion.y);
                prevMouseX = e.motion.x;
                prevMouseY = e.motion.y;
            }
        }

        const Uint8* keystates = SDL_GetKeyboardState(NULL);

        if(keystates[SDL_SCANCODE_LEFT]) {
            camera.translateVX(LEFT);
        }
        if(keystates[SDL_SCANCODE_RIGHT]) {
            camera.translateVX(RIGHT);
        }
        if(keystates[SDL_SCANCODE_UP]) {
            camera.translateVY(UP);
        }
        if(keystates[SDL_SCANCODE_DOWN]) {
            camera.translateVY(DOWN);
        }
        
        for(int i = 0; i < objects->size(); i++) {
            (*objects)[i]->draw(*this);
        }

        camera.update();

        SDL_RenderPresent(renderer);

        Uint32 currentTicks = SDL_GetTicks();
        frameCount++;
        if (currentTicks - prevTicks >= 1000) {
            int fps = frameCount * 1000 / (currentTicks - prevTicks);
            windowTitle = "c++ graphics - fps: " + std::to_string(fps);
            SDL_SetWindowTitle(window, windowTitle.c_str());
            
            frameCount = 0;
            prevTicks = currentTicks;
        }
        return 0;
    }

    void destroy() {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
};


void Polygon::draw(GraphicsWindow& graphics) {
		graphics.drawObject(*this);
	}

class RegularPolygon : public Polygon {
protected:
    int numSides;
    long double circumradius;

public:
    RegularPolygon(long double x = 0, long double y = 0, long double z = 0, int numSides = 5, long double circumradius = 50, Color color = Color())
    : numSides(numSides), circumradius(circumradius), Polygon(x, y, z, calculateRegularPolygonVertices(numSides, circumradius), color) {}

    long double getCircumradius() const {
        return circumradius;
    }

    int getNumSides() const {
        return numSides;
    }
};

class Polyhedron : public Polygon {
public:
    Polyhedron(long double x = 0, long double y = 0, long double z = 0, std::vector<std::array<long double, 3>> vertices = {}, long double yaw = 0, long double pitch = 0, long double roll = 0, Color color = Color())
    : Polygon(x,y,z,vertices,color,roll,pitch,yaw) {}
};

void setPixel(SDL_Renderer* renderer, long double x, long double y, Color color) {

    SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, (int) x, (int) y);
}

int main()
{

    std::vector<std::array<long double, 3>> cubeVerts {
        {-50.0L, -50.0L, -50.0L},
        {50.0L, -50.0L, -50.0L},
        {50.0L, 50.0L, -50.0L},
        {-50.0L, 50.0L, -50.0L},
        {-50.0L, -50.0L, 50.0L},
        {50.0L, -50.0L, 50.0L},
        {50.0L, 50.0L, 50.0L},
        {-50.0L, 50.0L, 50.0L}
    };
    std::vector<std::array<long double, 3>> squareVerts {
            {0.0L, 0.0L, 0.0L},
            {0.0L, -50.0L, 0.0L},
            {-50.0L, -50.0L, 0.0L},
            {-50.0L, 0.0L, 0.0L}
    };
    Color RED = Color(255,0,0);
    Polygon test(200.0L, 200.0L, 200.0L, squareVerts, RED);
    Polyhedron cube(400.0L, 400.0L, 100.0L, cubeVerts);
    RegularPolygon hexagon(300, 300, 200, 8, 50, Color(0, 255, 255));
    std::vector<Polygon*> objects{&test, &hexagon, &cube};
    GraphicsWindow window(800, 600, &objects, Color(255,255,255), Camera());
    
    window.open();

    while(window.isOpen()) {
        test.translateRoll(0.001);
        cube.translateRoll(0.001);
        cube.translatePitch(0.001);
        cube.translateYaw(0.001);
        cube.translate(0.1,0,0);
        window.refresh();
    }
}
