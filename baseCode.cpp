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
#define UP 1
#define RIGHT 1
#define DOWN -1


const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;


const std::vector<std::array<long double, 3>> cubeVerts {
    {-50.0L, -50.0L, -50.0L},
    {-50.0L, 50.0L, -50.0L},
    {-50.0L, 50.0L, 50.0L},
    {-50.0L, -50.0L, 50.0L},
    {50.0L, -50.0L, -50.0L},
    {50.0L, 50.0L, -50.0L},
    {50.0L, 50.0L, 50.0L},
    {50.0L, -50.0L, 50.0L}
};

std::vector<int> cubeVertsOrder {0,1,2,3,0,4,5,6,7,4,0,1,5,6,2,3,7};

class GraphicsWindow;
class Polyhedron;

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
        roll = std::fmod(roll + dRoll, 2*PI);
        // new slang when you notice the stripes!
    }

    void translatePitch(long double dPitch) {
        pitch = std::fmod(pitch + dPitch, 2*PI);
    }

    void translateYaw(long double dYaw) {
        yaw = std::fmod(yaw + dYaw, 2*PI);
    }

    void translateAngle(long double dRoll, long double dPitch, long double dYaw) {
        roll = std::fmod(roll + dRoll, 2*PI);
        pitch = std::fmod(pitch + dPitch, 2*PI);
        yaw = std::fmod(yaw + dYaw, 2*PI);
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
           long double movementSpeed = 1, long double lookSpeed = 0.001L,
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
        translatePitch(x * lookSpeed);
        //translateRoll(y * lookSpeed);
        //std::cout << roll << std::endl;
        //if(roll > PI/2) {
        //    setRoll(PI/2);
        //} else if(roll < -PI/2) {
        //    setRoll(-PI/2);
        //}
    }
};

std::array<long double, 3> rotateVertex(long double x, long double y, long double z, long double roll, long double pitch, long double yaw) {
    long double cosyaw = cos(yaw);
    long double cospitch = cos(pitch);
    long double cosroll = cos(roll);
    long double sinyaw = sin(yaw);
    long double sinpitch = sin(pitch);
    long double sinroll = sin(roll);

    // dot product of vector [x,y,z] and product of rotation matrix Z * rotation matrix Y * rotation matrix X
    std::array<long double, 3> rotatedVector {
        x*cosyaw*cospitch+y*cosyaw*sinpitch*sinroll-y*sinyaw*cosroll+z*cosyaw*sinpitch*cosroll+z*sinyaw*sinroll,
        x*sinyaw*cospitch+y*sinyaw*sinpitch*sinroll+y*cosyaw*cosroll+z*sinyaw*sinpitch*cosroll-z*cosyaw*sinroll,
        x*-sinpitch+y*cospitch*sinroll+z*cospitch*cosroll
    };

    return rotatedVector;
}

class Polyhedron : public GraphicalObject {
protected:
	std::vector<std::array<long double, 3>> vertices;
    std::vector<int> vertsOrder;
    Color color;

public:
	Polyhedron(long double x = 0, long double y = 0, long double z = 0, std::vector<std::array<long double, 3>> vertices = {}, Color color = Color(), std::vector<int> vertsOrder = {}, long double roll = 0, long double pitch = 0, long double yaw = 0)
    : GraphicalObject(x, y, z, roll, pitch, yaw), vertices(vertices), vertsOrder(verifyVertsOrder(vertsOrder)), color(color) {}

    std::vector<int> verifyVertsOrder(std::vector<int> vertsOrder) {
        if(vertsOrder.size() != 0) {
            return vertsOrder;
        }
        return defaultVertsOrder();
    }

    std::vector<int> defaultVertsOrder() {
        std::vector<int> vertsOrder;
        for(int i = 0; i < vertices.size(); i++) {
            vertsOrder.push_back(i);
        }
        vertsOrder.push_back(0);
        return vertsOrder;
    }

    void draw(GraphicsWindow& graphics);


    Color& getColor() {
        return color;
    }

    std::vector<int>& getVertsOrder() {
        return vertsOrder;
    }

    std::vector<std::array<long double, 3>>& getVertices() {
        return vertices;
    }
};


class GraphicsWindow {
private:
    int screenWidth;
    int screenHeight;
    std::vector<Polyhedron*>* objects;
    Color backgroundColor;
    SDL_Window* window;
    SDL_Renderer* renderer;
    Uint32 prevTicks;
    bool quit;
    SDL_Event e;
    int frameCount;
    std::string windowTitle;
    Camera camera;
    Color RED;
    Color GREEN;
    Color BLUE;

public:
    GraphicsWindow(int screenWidth, int screenHeight, std::vector<Polyhedron*>* objects, Color backgroundColor, Camera camera)
    : screenWidth(screenWidth), screenHeight(screenHeight), objects(objects), backgroundColor(backgroundColor), camera(camera), RED(Color(255,0,0)), GREEN(Color(0,255,0)), BLUE(Color(0,0,255)) {}


    void drawObjectTwoWowTheseFunctionsAreOrganizedVeryPoorlyFixThisLaterPlease(long double x1, long double y1, long double z1, long double x2, long double y2, long double z2, Color& color) {
        std::array<long double, 3> v1 = rotateVertex(x1 - camera.getX(), y1 - camera.getY(), z1 - camera.getZ(), camera.getRoll(), camera.getPitch(), camera.getYaw());
        std::array<long double, 3> v2 = rotateVertex(x2 - camera.getX(), y2 - camera.getY(), z2 - camera.getZ(), camera.getRoll(), camera.getPitch(), camera.getYaw());

        long double newX1 = (v1[0] / (v1[2]*0.001));
        long double newX2 = (v2[0] / (v2[2]*0.001));
        long double newY1 = (v1[1] / (v1[2]*0.001));
        long double newY2 = (v2[1] / (v2[2]*0.001));
         
        // if line is off screen, do not display
        if((newX1 < 0 || newX1 > SCREEN_WIDTH) && (newX2 < 0 || newX2 > SCREEN_WIDTH)
        || (newY1 < 0 || newY1 > SCREEN_HEIGHT) && (newY2 < 0 || newY2 > SCREEN_HEIGHT)
        || v1[2] < 0 && v2[2] < 0) {
            return;
        }

        SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
        drawLine((int) newX1, (int) newY1, (int) newX2, (int) newY2, color);
        //SDL_RenderDrawLine(renderer, (int) ((x1 - camera.getX())/(z1 - camera.getZ())), (int) ((y1 - camera.getY())/(z1 - camera.getZ())), (int) ((x2 - camera.getX())/(z2 - camera.getZ())), (int) ((y2 - camera.getY())/(z2 - camera.getZ())));
    }

    void drawLine(long double x1, long double y1, long double x2, long double y2, Color& color) {
        SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
        SDL_RenderDrawLine(renderer, (int) x1, (int) y1, (int) x2, (int) y2);
    }

    void drawObject(Polyhedron& object) {
        std::vector<std::array<long double, 3>> rotatedVertices{};
        std::vector<std::array<long double, 3>>& verticesPointer = object.getVertices();
        for(int i = 0; i < verticesPointer.size(); i++) {
            std::array<long double, 3> tempVerts = rotateVertex(verticesPointer[i][0],verticesPointer[i][1],verticesPointer[i][2],object.getRoll(),object.getPitch(),object.getYaw());
            rotatedVertices.push_back(tempVerts);
        }
        std::vector<int>& vertsOrderPointer = object.getVertsOrder();
        for(int i = 0; i < vertsOrderPointer.size() - 1; i++) {
            this->drawObjectTwoWowTheseFunctionsAreOrganizedVeryPoorlyFixThisLaterPlease(rotatedVertices[vertsOrderPointer[i]][0] + object.getX(), rotatedVertices[vertsOrderPointer[i]][1] + object.getY(), rotatedVertices[vertsOrderPointer[i]][2] + object.getZ(), rotatedVertices[vertsOrderPointer[i+1]][0] + object.getX(), rotatedVertices[vertsOrderPointer[i+1]][1] + object.getY(), rotatedVertices[vertsOrderPointer[i+1]][2] + object.getZ(), object.getColor());
        }
    }

    bool isOpen() {
        return !quit;
    }

    void addObject(Polyhedron* object) {
        objects->push_back(object);
    }

    int open() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            return 1;
        }

        window = SDL_CreateWindow("c++ graphics - fps:", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, screenWidth, screenHeight, SDL_WINDOW_SHOWN);
        SDL_SetRelativeMouseMode(SDL_TRUE);
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
                camera.translateLook(-e.motion.xrel, e.motion.yrel);
                //std::cout << "xrel: " << e.motion.xrel << " yrel: " << e.motion.yrel << std::endl;
            }
            if (e.type == SDL_KEYDOWN) {
                SDL_Keycode keyPressed = e.key.keysym.sym;
                if (keyPressed == SDLK_g) {
                    Polyhedron* poly = new Polyhedron(camera.getX(), camera.getY() + 300, camera.getZ(), cubeVerts, Color(), cubeVertsOrder);
                    addObject(poly);
                }
            }
        }

        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        

        const Uint8* keystates = SDL_GetKeyboardState(NULL);

        if(keystates[SDL_SCANCODE_LEFT]) {
            camera.translateVX(LEFT);
        }
        if(keystates[SDL_SCANCODE_RIGHT]) {
            camera.translateVX(RIGHT);
        }
        if(keystates[SDL_SCANCODE_UP]) {
            camera.translateVZ(UP);
        }
        if(keystates[SDL_SCANCODE_DOWN]) {
            camera.translateVZ(DOWN);
        }
        if(keystates[SDL_SCANCODE_Q]) {
            destroy();
        }

        for(int i = 0; i < objects->size(); i++) {
            (*objects)[i]->draw(*this);
        }

        camera.update();

        std::array<long double, 3> rotatedXVertex = rotateVertex(50,0,0,camera.getRoll(), camera.getPitch(), camera.getYaw());
        std::array<long double, 3> rotatedYVertex = rotateVertex(0,50,0,camera.getRoll(), camera.getPitch(), camera.getYaw());
        std::array<long double, 3> rotatedZVertex = rotateVertex(0,0,50,camera.getRoll(), camera.getPitch(), camera.getYaw());
        drawLine(60,50,rotatedXVertex[0]+60, rotatedXVertex[1]+60, RED);
        drawLine(60,50,rotatedYVertex[0]+60, rotatedYVertex[1]+60, GREEN);
        drawLine(60,50,rotatedZVertex[0]+60, rotatedZVertex[1]+60, BLUE);

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
        quit=true;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
};


void Polyhedron::draw(GraphicsWindow& graphics) {
		graphics.drawObject(*this);
}

class RegularPolygon : public Polyhedron {
protected:
    int numSides;
    long double circumradius;

public:
    RegularPolygon(long double x = 0, long double y = 0, long double z = 0, int numSides = 5, long double circumradius = 50, Color color = Color())
    : numSides(numSides), circumradius(circumradius), Polyhedron(x, y, z, calculateRegularPolygonVertices(numSides, circumradius), color) {}

    long double getCircumradius() const {
        return circumradius;
    }

    int getNumSides() const {
        return numSides;
    }
};

void setPixel(SDL_Renderer* renderer, long double x, long double y, Color color) {

    SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, (int) x, (int) y);
}

int main()
{
    std::vector<std::array<long double, 3>> squareVerts {
            {0.0L, 0.0L, 0.0L},
            {0.0L, -50.0L, 0.0L},
            {-50.0L, -50.0L, 0.0L},
            {-50.0L, 0.0L, 0.0L}
    };


    Color RED = Color(255,0,0);
    Polyhedron test(200.0L, 200.0L, 200.0L, squareVerts, RED);
    Polyhedron cube(400.0L, 300.0L, 100.0L, cubeVerts, Color(), cubeVertsOrder);
    RegularPolygon hexagon(300, 300, 200, 8, 50, Color(0, 255, 255));
    std::vector<Polyhedron*> objects{&test, &hexagon, &cube};
    GraphicsWindow window(800, 600, &objects, Color(255,255,255), Camera(0,0,-1000,0,0,0));
    
    window.open();
    int direction = 1;
    long double speed = 0.001;
    while(window.isOpen()) {
        //test.translateRoll(speed);
        //speed *= 1.0001;
        hexagon.translatePitch(-0.005);
        //cube.translateAngle(0.001,0.001,0.001);
        //if(cube.getX() > 600 || cube.getX() < 200) direction *= -1;
        //cube.translate(direction * 0.1, 0, 0);
        window.refresh();
    }
}
