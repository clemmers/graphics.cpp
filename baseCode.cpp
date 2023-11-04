// g++ -o baseCode baseCode.cpp -lSDL2 -lGL -lGLU -lglut -I/usr/include/SDL2 -L/usr/lib

#include <SDL2/SDL.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <array>
#include <chrono>
#include <unordered_map>
#include <utility>

#define PI 3.141592653589793238462643383279502884L
#define LEFT -1
#define UP 1
#define RIGHT 1
#define DOWN -1


const std::vector<std::array<long double, 3>> cubeVerts {
    {-50.0L, -50.0L, -50.0L}, // 0
    {-50.0L, 50.0L, -50.0L},  // 1
    {-50.0L, 50.0L, 50.0L},   // 2
    {-50.0L, -50.0L, 50.0L},  // 3
    {50.0L, -50.0L, -50.0L},  // 4
    {50.0L, 50.0L, -50.0L},   // 5
    {50.0L, 50.0L, 50.0L},    // 6
    {50.0L, -50.0L, 50.0L}    // 7
};

std::vector<int> cubeVertsOrder {0,1,2,3,0,4,5,6,7,4,0,1,5,6,2,3,7};

std::vector<std::vector<int>> cubeFaces {
    {0,1,2,3,0},
    {0,4,5,1,0},
    {4,5,6,7,4},
    {7,6,2,3,7},
    {1,5,6,2,1},
    {0,4,7,3,0}
};

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

    void setColor(int red, int green, int blue) {
        this->red = red;
        this->green = green;
        this->blue = blue;
    }
};

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
        //roll = std::fmod(roll + dRoll, 2*PI);
        roll += dRoll;
        // new slang when you notice the stripes!
    }

    void translatePitch(long double dPitch) {
        //pitch = std::fmod(pitch + dPitch, 2*PI);
        pitch += dPitch;
    }

    void translateYaw(long double dYaw) {
        //yaw = std::fmod(yaw + dYaw, 2*PI);
        yaw += dYaw;
    }

    void translateAngle(long double dRoll, long double dPitch, long double dYaw) {
        //roll = std::fmod(roll + dRoll, 2*PI);
        //pitch = std::fmod(pitch + dPitch, 2*PI);
        //yaw = std::fmod(yaw + dYaw, 2*PI);

        roll += dRoll;
        pitch += dPitch;
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
           long double movementSpeed = 1, long double lookSpeed = 0.001L,
           long double friction = 0.5, long double vX = 0, long double vY = 0,
           long double vZ = 0)
        : GraphicalObject(x,y,z,roll,pitch,yaw), friction(friction),
          movementSpeed(movementSpeed), lookSpeed(lookSpeed),
          vX(vX), vY(vY), vZ(vZ),
          timeWhenLastUpdated(std::chrono::high_resolution_clock::now()) {}

    void updatePosition() {
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

    // dot product of product of rotation matrix Z * rotation matrix Y * rotation matrix X and vector [x,y,z]
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
    std::vector<std::vector<int>> faces;
    Color outlineColor;
    Color color;

public:
	Polyhedron(long double x = 0, long double y = 0, long double z = 0, std::vector<std::array<long double, 3>> vertices = {}, Color color = Color(255,0,220), Color outlineColor = Color(), std::vector<std::vector<int>> faces = {}, std::vector<int> vertsOrder = {}, long double roll = 0, long double pitch = 0, long double yaw = 0)
    : GraphicalObject(x, y, z, roll, pitch, yaw), vertices(vertices), vertsOrder(verifyVertsOrder(vertsOrder)), faces(verifyFaces(faces)), color(color), outlineColor(outlineColor) {}

    std::vector<int> verifyVertsOrder(std::vector<int> vertsOrder) {
        if(vertsOrder.size() != 0) {
            return vertsOrder;
        }
        return defaultVertsOrder();
    }

    std::vector<std::vector<int>> verifyFaces(std::vector<std::vector<int>> faces) {
        if(faces.size() != 0) {
            return faces;
        }
        faces.push_back(defaultVertsOrder());
        return faces;
    }

    std::vector<int> defaultVertsOrder() {
        std::vector<int> vertsOrder;
        for(int i = 0; i < vertices.size(); i++) {
            vertsOrder.push_back(i);
        }
        vertsOrder.push_back(0);
        return vertsOrder;
    }


    Color& getOutlineColor() {
        return outlineColor;
    }

    Color& getColor() {
        return color;
    }

    std::vector<int>& getVertsOrder() {
        return vertsOrder;
    }

    std::vector<std::vector<int>>& getFaces() {
        return faces;
    }

    std::vector<std::array<long double, 3>>& getVertices() {
        return vertices;
    }
};

class RegularPolygon : public Polyhedron {
protected:
    int numSides;
    long double circumradius;

    std::vector<std::array<long double, 3>> calculateRegularPolygonVertices(int numSides, long double circumradius) {
        std::vector<std::array<long double, 3>> vertices {};
        long double angle = 2 * PI / numSides;
        for (int i = 0; i < numSides; i++)
        {
            vertices.push_back(std::array<long double, 3>{circumradius * cos(i * angle), circumradius * sin(i * angle), 0});
        }
        return vertices;
    }

public:
    RegularPolygon(long double x = 0, long double y = 0, long double z = 0, int numSides = 5, long double circumradius = 50, Color color = Color(255,0,220), Color outlineColor = Color())
    : numSides(numSides), circumradius(circumradius), Polyhedron(x, y, z, calculateRegularPolygonVertices(numSides, circumradius), color, outlineColor) {}

    long double getCircumradius() const {
        return circumradius;
    }

    int getNumSides() const {
        return numSides;
    }
};


class Cuboid : public Polyhedron {
protected:
    long double width;
    long double height;
    long double depth;

    
    std::vector<std::array<long double, 3>> calculateCuboidVertices(long double width, long double height, long double depth) {
        long double halfWidth = width/2.0L;
        long double halfHeight = height/2.0L;
        long double halfDepth = depth/2.0L;
        
        std::vector<std::array<long double, 3>> cuboidVerts {
            {-halfWidth, -halfHeight, -halfDepth}, // 0
            {-halfWidth,  halfHeight, -halfDepth}, // 1
            {-halfWidth,  halfHeight,  halfDepth}, // 2
            {-halfWidth, -halfHeight,  halfDepth}, // 3
            { halfWidth, -halfHeight, -halfDepth}, // 4
            { halfWidth,  halfHeight, -halfDepth}, // 5
            { halfWidth,  halfHeight,  halfDepth}, // 6
            { halfWidth, -halfHeight,  halfDepth}  // 7
        };

        return cuboidVerts;
    };




public:
    Cuboid(long double x = 0, long double y = 0, long double z = 0, long double width = 100, long double height = 100, long double depth = 100, Color color = Color(255,0,220), Color outlineColor = Color())
    : width(width), height(height), depth(depth), Polyhedron(x, y, z, calculateCuboidVertices(width, height, depth), color, outlineColor, std::vector<std::vector<int>> {{0,1,2,3,0},{0,4,5,1,0},{4,5,6,7,4},{7,6,2,3,7},{1,5,6,2,1},{0,4,7,3,0}}, std::vector<int> {0,1,2,3,0,4,5,6,7,4,0,1,5,6,2,3,7}) {}
};

class Cube : public Cuboid {
protected:
    long double circumradius;
    
public:
    Cube(long double x = 0, long double y = 0, long double z = 0, long double circumradius = 100, Color color = Color(255,0,220), Color outlineColor = Color())
    : circumradius(circumradius), Cuboid(x,y,z,circumradius,circumradius,circumradius,color,outlineColor) {};
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
    std::unordered_map<std::string, long double> pixelDepth;
    bool isWireframe;

public:
    GraphicsWindow(int screenWidth, int screenHeight, std::vector<Polyhedron*>* objects, Color backgroundColor, Camera camera)
    : screenWidth(screenWidth), screenHeight(screenHeight), objects(objects), backgroundColor(backgroundColor), camera(camera), RED(Color(255,0,0)), GREEN(Color(0,255,0)), BLUE(Color(0,0,255)), isWireframe(false) {}

    void setPixel(int x, int y, long double z, Color color) {
        // dont bother with off-screen pixels
        if(x < 0 || x > screenWidth - 1 || y < 0 || y > screenHeight - 1 || z < 0) {
            return;
        }
        std::string key = std::to_string(x) + " " + std::to_string(y);
        // the closest pixel to the camera gets drawn
        if(pixelDepth.find(key) != pixelDepth.end() && pixelDepth[key] < z) {
            return;
        }
        SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
        SDL_RenderDrawPoint(renderer, x, y);
        pixelDepth[key] = z;
    }

    void clearScreen() {
        SDL_SetRenderDrawColor(renderer, backgroundColor.getRed(), backgroundColor.getGreen(), backgroundColor.getBlue(), 255);
        SDL_RenderClear(renderer);
        pixelDepth.clear();
    }

    
    void setLine(long double x1, long double y1, long double z1, long double x2, long double y2, long double z2, Color& color) {
        if((int) (x1 + 0.5) == (int) (x2 + 0.5)) {
            if(y1 > y2) {
                long double tempX = x1;
                long double tempY = y1;
                long double tempZ = z1;
                x1 = x2;
                y1 = y2;
                z1 = z2;
                x2 = tempX;
                y2 = tempY;
                z2 = tempZ;
            }
            long double yzSlope = (z2 - z1) / (y2 - y1);
            x1 = (int) (x1 + 0.5);
            long double curZ = z1;
            for(int i = (int) (y1 + 0.5); i < (int) (y2 + 0.5); i++) {
                curZ += yzSlope;
                setPixel(x1, i, curZ, color);
            }
            return;
        }
        if(abs(y2 - y1) < abs(x2 - x1)) {
            if(x1 > x2) {
                long double tempX = x1;
                long double tempY = y1;
                long double tempZ = z1;
                x1 = x2;
                y1 = y2;
                z1 = z2;
                x2 = tempX;
                y2 = tempY;
                z2 = tempZ;
            }
            long double xySlope = (y2 - y1) / (x2 - x1);
            long double xzSlope = (z2 - z1) / (x2 - x1);
            long double curY = y1;
            long double curZ = z1;
            for(int i = (x1 + 0.5); i < x2; i++) {
                curY += xySlope;
                curZ += xzSlope;
                setPixel(i, (int) (curY + 0.5), curZ, color);
            }
            return;
        }
        if(y1 > y2) {
            long double tempX = x1;
            long double tempY = y1;
            long double tempZ = z1;
            x1 = x2;
            y1 = y2;
            z1 = z2;
            x2 = tempX;
            y2 = tempY;
            z2 = tempZ;
        }
        long double yxSlope = (x2 - x1) / (y2 - y1);
        long double yzSlope = (z2 - z1) / (y2 - y1);
        long double curX = x1;
        long double curZ = z1;
        for(int i = (y1 + 0.5); i < y2; i++) {
            curX += yxSlope;
            curZ += yzSlope;
            setPixel((int) (curX + 0.5), i, curZ, color);
        }
        return;
    }


    void setEdge(long double x1, long double y1, long double z1, long double x2, long double y2, long double z2, std::unordered_map<int, std::vector<std::pair<int, long double>>>* face, Color& color) {
        if((int) (x1 + 0.5) == (int) (x2 + 0.5)) {
            if(y1 > y2) {
                long double tempX = x1;
                long double tempY = y1;
                long double tempZ = z1;
                x1 = x2;
                y1 = y2;
                z1 = z2;
                x2 = tempX;
                y2 = tempY;
                z2 = tempZ;
            }
            long double yzSlope = (z2 - z1) / (y2 - y1);
            x1 = (int) (x1 + 0.5);
            long double curZ = z1;
            for(int i = (int) (y1 + 0.5); i < (int) (y2 + 0.5); i++) {
                curZ += yzSlope;
                (*face)[i].push_back({x1, curZ});
            }
            return;
        }
        
        if(abs(y2 - y1) < abs(x2 - x1)) {
            if(x1 > x2) {
                long double tempX = x1;
                long double tempY = y1;
                long double tempZ = z1;
                x1 = x2;
                y1 = y2;
                z1 = z2;
                x2 = tempX;
                y2 = tempY;
                z2 = tempZ;
            }
            long double xySlope = (y2 - y1) / (x2 - x1);
            long double xzSlope = (z2 - z1) / (x2 - x1);
            long double curY = y1;
            long double curZ = z1;
            int prevY = (int) (curY + 0.5);
            for(int i = (x1 + 0.5); i < x2; i++) {
                curY += xySlope;
                curZ += xzSlope;
                if((int) (curY + 0.5) == prevY) {
                    continue;
                }
                prevY = (int) (curY + 0.5);
                (*face)[prevY].push_back({i, curZ});
            }
            return;
        }
        
        if(y1 > y2) {
            long double tempX = x1;
            long double tempY = y1;
            long double tempZ = z1;
            x1 = x2;
            y1 = y2;
            z1 = z2;
            x2 = tempX;
            y2 = tempY;
            z2 = tempZ;
        }
        long double yxSlope = (x2 - x1) / (y2 - y1);
        long double yzSlope = (z2 - z1) / (y2 - y1);
        long double curX = x1;
        long double curZ = z1;
        for(int i = (y1 + 0.5); i < y2; i++) {
            curX += yxSlope;
            curZ += yzSlope;
            (*face)[i].push_back({(int) (curX + 0.5), curZ});
        }
        return;
    }

    // draws a HORIZONTAL LINE
    void setFillLine(int y, int x1, long double z1, int x2, long double z2, Color& color) {
        if(x1 > x2) {
            int tempX = x1;
            long double tempZ = z1;
                x1 = x2;
                z1 = z2;
                x2 = tempX;
                z2 = tempZ;
        }
        long double xzSlope = (z2 - z1) / (x2 - x1);
        for(int i = x1; i < x2; i++) {
            setPixel(i, y, z1, color);
            z1 += xzSlope;
        }
    }

    void setScreenCoordinates(Polyhedron& object) {
        std::vector<std::array<long double, 3>> rotatedVertices{};
        
        std::vector<std::array<long double, 3>>& verticesPointer = object.getVertices();
        for(int i = 0; i < verticesPointer.size(); i++) {
            std::array<long double, 3> tempVerts = rotateVertex(verticesPointer[i][0],verticesPointer[i][1],verticesPointer[i][2],object.getRoll(),object.getPitch(),object.getYaw());
            tempVerts = rotateVertex(tempVerts[0] + object.getX() - camera.getX(), tempVerts[1] + object.getY() - camera.getY(), tempVerts[2] + object.getZ() - camera.getZ(), camera.getRoll(), camera.getPitch(), camera.getYaw());
            tempVerts[0] = tempVerts[0] / (tempVerts[2]*0.001);
            tempVerts[1] = tempVerts[1] / (tempVerts[2]*0.001);
            rotatedVertices.push_back(tempVerts);
        }
        std::vector<int>& vertsOrderPointer = object.getVertsOrder();
        std::vector<std::vector<int>>& facesPointer = object.getFaces();

        // for each face
        for(int p = 0; p < facesPointer.size(); p++) {

            std::unordered_map<int, std::vector<std::pair<int, long double>>> faceEdges;
            int minX = (int) (rotatedVertices[facesPointer[p][0]][0] + 0.5);
            int maxX = minX;
            int minY = (int) (rotatedVertices[facesPointer[p][0]][1] + 0.5);
            int maxY = minY;

            long double x1 = rotatedVertices[facesPointer[p][0]][0];
            long double y1 = rotatedVertices[facesPointer[p][0]][1];
            long double z1 = rotatedVertices[facesPointer[p][0]][2];
            long double x2 = x1;
            long double y2 = y1;
            long double xzSlope = 0;
            long double yzSlope = 0;

            // for each vertex in face
            for(int i = 0; i < facesPointer[p].size() - 1; i++) {
                if(rotatedVertices[facesPointer[p][i+1]][0] != x1 && x1 == x2) {
                    x2 = rotatedVertices[facesPointer[p][i+1]][0];
                    xzSlope = (rotatedVertices[facesPointer[p][i+1]][2] - z1) / (x2 - x1);
                }
                if(rotatedVertices[facesPointer[p][i+1]][1] != y1 && y1 == y2) {
                    y2 = rotatedVertices[facesPointer[p][i+1]][1];
                    yzSlope = (rotatedVertices[facesPointer[p][i+1]][2] - z1) / (y2 - y1);
                }
                minX = std::min(minX, (int) (rotatedVertices[facesPointer[p][i+1]][0] + 0.5));
                maxX = std::max(maxX, (int) (rotatedVertices[facesPointer[p][i+1]][0] + 0.5));
                minY = std::min(minY, (int) (rotatedVertices[facesPointer[p][i+1]][1] + 0.5));
                maxY = std::max(maxY, (int) (rotatedVertices[facesPointer[p][i+1]][1] + 0.5));

                setEdge(rotatedVertices[facesPointer[p][i]][0], rotatedVertices[facesPointer[p][i]][1], rotatedVertices[facesPointer[p][i]][2], rotatedVertices[facesPointer[p][i+1]][0], rotatedVertices[facesPointer[p][i+1]][1], rotatedVertices[facesPointer[p][i+1]][2], &faceEdges, object.getOutlineColor());
            }

            long double z = rotatedVertices[facesPointer[p][0]][2] + (minY - rotatedVertices[facesPointer[p][0]][1]) * yzSlope + (minX - rotatedVertices[facesPointer[p][0]][0]) * xzSlope;
            
            bool isInFace = false;


            if(!isWireframe) {
                //std::cout << z << " z" << std::endl;
                //std::cout << xzSlope << " xz slope" << std::endl;
                //std::cout << yzSlope << " yz slope" << std::endl;

                for (const auto & [ y, xzVals ] : faceEdges) {
                    if (xzVals.size() != 2) {
                        if(xzVals.size() > 2) {
                            for (auto i: xzVals) {
                                std::cout << i.first << ' ';
                            }
                            std::cout << "is there a concave face? may not be filled correctly" << std::endl;
                            
                        }
                        continue;
                    }
                    setFillLine(y, xzVals[0].first, xzVals[0].second, xzVals[1].first, xzVals[1].second, object.getColor());
                }
            }
        }

        for(int i = 0; i < vertsOrderPointer.size() - 1; i++) {
            setLine(rotatedVertices[vertsOrderPointer[i]][0], rotatedVertices[vertsOrderPointer[i]][1], rotatedVertices[vertsOrderPointer[i]][2], rotatedVertices[vertsOrderPointer[i+1]][0], rotatedVertices[vertsOrderPointer[i+1]][1], rotatedVertices[vertsOrderPointer[i+1]][2], object.getOutlineColor());
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
        clearScreen();
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
                    Cube* poly = new Cube(camera.getX(), camera.getY() + 300, camera.getZ(), 100);
                    addObject(poly);
                }
                if (keyPressed == SDLK_f) {
                    isWireframe = !isWireframe;
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
            setScreenCoordinates(*((*objects)[i]));
        }

        camera.updatePosition();

        std::array<long double, 3> rotatedXVertex = rotateVertex(50,0,0,camera.getRoll(), camera.getPitch(), camera.getYaw());
        std::array<long double, 3> rotatedYVertex = rotateVertex(0,50,0,camera.getRoll(), camera.getPitch(), camera.getYaw());
        std::array<long double, 3> rotatedZVertex = rotateVertex(0,0,50,camera.getRoll(), camera.getPitch(), camera.getYaw());
        setLine(60,50,60,rotatedXVertex[0]+60, rotatedXVertex[1]+60, rotatedXVertex[2]+60, RED);
        setLine(60,50,60,rotatedYVertex[0]+60, rotatedYVertex[1]+60, rotatedYVertex[2]+60, GREEN);
        setLine(60,50,60,rotatedZVertex[0]+60, rotatedZVertex[1]+60, rotatedZVertex[2]+60, BLUE);

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
    Cube cube(400, 300, 100, 100);
    RegularPolygon hexagon(275, 300, 100, 8, 50, Color(0, 255, 255));

    //Polyhedron floor(0,350,0,std::vector<std::array<long double, 3>>{{-3000,0,-3000},{-3000,0,3000},{3000,0,3000},{3000,0,-3000}});

    std::vector<Polyhedron*> objects{&test, &hexagon, &cube/*, &floor*/};
    GraphicsWindow window(800, 400, &objects, Color(255,255,255), Camera(0,0,-1000,0,0,0));
    
    window.open();
    int direction = 1;
    long double speed = 0.001;
    while(window.isOpen()) {
        //test.translateRoll(speed);
        //speed *= 1.0001;
        //hexagon.translatePitch(-0.005);
        //cube.translateAngle(0.001,0.001,0.001);
        //if(cube.getX() > 600 || cube.getX() < 200) direction *= -1;
        //cube.translate(direction * 0.1, 0, 0);
        window.refresh();
    }
}
