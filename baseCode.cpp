// g++ -o baseCode baseCode.cpp -lSDL2 -lGL -lGLU -lglut -I/usr/include/SDL2 -L/usr/lib

#include <SDL2/SDL.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <array>

#define PI 3.141592653589793238462643383279502884L


const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

class GraphicsWindow;

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

    void setRed(int r) {
        this->red = red;
    }

    void setGreen(int green) {
        this->green = green;
    }

    void setBlue(int blue) {
        this->blue = blue;
    }
};

std::vector<std::vector<long double>> calculateRegularPolygonVertices (int numSides, long double circumradius) {
    std::vector<std::vector<long double>> vertices {};
        long double angle = 2 * PI / numSides;
        for (int i = 0; i < numSides; i++)
        {
            vertices.push_back(std::vector<long double>{circumradius * cos(i * angle), circumradius * sin(i * angle)});
        }
        return vertices;
}

class TwoDimensionalObject {
protected:
	long double x;
	long double y;
    Color color;

public:

    virtual void draw(GraphicsWindow* graphics) const = 0;
    TwoDimensionalObject(long double xx, long double yy, Color colorr)
	: x(xx), y(yy), color(colorr) {}

    Color* getColorPointer() {
        return &color;
    }

    int getX() const {
        return x;
    }
    
    int getY() const {
        return y;
    }

    void setX(long double x) {
        this->x = x;
    }
    
    void setY(long double y) {
        this->y = y;
    }

    void transform(long double dx, long double dy) {
        this->x += dx;
        this->y += dy;
    }
};

class Camera {
private:
    std::array<long double, 3> XYZ;
    std::array<long double, 3> rollPitchYaw;
    std::array<long double, 3> vXYZ;
    std::array<long double, 3> aXYZ;
    std::array<long double, 3> vRollPitchYaw;
    std::array<long double, 3> aRollPitchYaw;

public:
    Camera(const std::array<long double, 3>& XYZ = {},
           const std::array<long double, 3>& rollPitchYaw = {},
           const std::array<long double, 3>& vXYZ = {},
           const std::array<long double, 3>& aXYZ = {},
           const std::array<long double, 3>& vRollPitchYaw = {},
           const std::array<long double, 3>& aRollPitchYaw = {})
        : XYZ(XYZ),
          rollPitchYaw(rollPitchYaw),
          vXYZ(vXYZ),
          aXYZ(aXYZ),
          vRollPitchYaw(vRollPitchYaw),
          aRollPitchYaw(aRollPitchYaw) {}

    long double getX() const {
        return XYZ[0];
    }

    long double getY() const {
        return XYZ[1];
    }

    long double getZ() const {
        return XYZ[2];
    }
};


class GraphicsWindow {
private:
    int screenWidth;
    int screenHeight;
    std::vector<TwoDimensionalObject*> objects;
    Color backgroundColor;
    SDL_Window* window;
    SDL_Renderer* renderer;
    Uint32 prevTicks;
    bool quit;
    SDL_Event e;
    int frameCount;
    std::string windowTitle;
    Camera camera;

public:
    GraphicsWindow(int screenWidthInput, int screenHeightInput, std::vector<TwoDimensionalObject*> objectss, Color backgroundColorInput, Camera cameraInput)
    : screenWidth(screenWidthInput), screenHeight(screenHeightInput), objects(objectss), backgroundColor(backgroundColorInput), camera(cameraInput) {}

    GraphicsWindow(int screenWidthInput, int screenHeightInput, std::vector<TwoDimensionalObject*> objectss)
    : screenWidth(screenWidthInput), screenHeight(screenHeightInput), objects(objectss), backgroundColor(Color()), camera(Camera()) {}

    void drawLine(long double x1, long double y1, long double x2, long double y2, Color color) {
        SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
        SDL_RenderDrawLine(renderer, (int) (x1 - camera.getX()), (int) (y1 - camera.getY()), (int) (x2 - camera.getX()), (int) (y2 - camera.getY()));
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
        if (!quit) {
            SDL_SetRenderDrawColor(renderer, backgroundColor.getRed(), backgroundColor.getGreen(), backgroundColor.getBlue(), 255);
            SDL_RenderClear(renderer);
            while (SDL_PollEvent(&e) != 0) {
                if (e.type == SDL_QUIT) {
                    quit = true;
                } else if(e.type == SDL_KEYDOWN) {
                    switch (e.key.keysym.sym) {
                        case SDLK_UP:
                            std::cout << "up!" << std::endl;
                            break;
                        case SDLK_DOWN:
                            break;
                        case SDLK_LEFT:
                            break;
                        case SDLK_RIGHT:
                            break;
                        default:
                            break;
                    }
                }
            }

            for(int i = 0; i < objects.size(); i++) {
                objects[i]->draw(this);
            }

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

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();

        return 1;
    }

    void destroy() {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
};



class Polygon : public TwoDimensionalObject {
protected:
	std::vector<std::vector<long double>> vertices;

public:
	Polygon(long double x, long double y, std::vector<std::vector<long double>> verticess, Color color)
    : TwoDimensionalObject(x, y, color), vertices(verticess) {}

    Polygon(std::vector<std::vector<long double>> verticess) : TwoDimensionalObject(0.0L, 0.0L, Color()), vertices(verticess) {}

    std::vector<std::vector<long double>>* getVerticesPointer() {
        return &vertices;
    }

	void draw(GraphicsWindow* graphics) const override {
		int i;
		for(i = 0; i < vertices.size() - 1; i++) {
			graphics->drawLine(vertices[i][0] + x, vertices[i][1] + y, vertices[i+1][0] + x, vertices[i+1][1] + y, color);
		}
		graphics->drawLine(vertices[i][0] + x, vertices[i][1] + y, vertices[0][0] + x, vertices[0][1] + y, color);
	}

    void rotate(long double angle) {

        long double rotationMatrix[2][2] = {
            { cos(angle), -sin(angle) },
            { sin(angle),  cos(angle) }
        };

        for(int i = 0; i < vertices.size(); i++) {
            vertices[i][0] = vertices[i][0] * rotationMatrix[0][0] + vertices[i][1] * rotationMatrix[0][1];
            vertices[i][1] = vertices[i][0] * rotationMatrix[1][0] + vertices[i][1] * rotationMatrix[1][1];
        }
    }
};

class RegularPolygon : public Polygon {
protected:
    int numSides;
    long double circumradius;

public:
    RegularPolygon(long double x, long double y, int numSidess, long double circumradiuss, Color color)
    : numSides(numSidess), circumradius(circumradiuss), Polygon(x, y, calculateRegularPolygonVertices(numSidess, circumradiuss), color) {}

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

    std::vector<std::vector<long double>> verts {
            {0.0L, 0.0L},
            {0.0L, -50.0L},
            {-50.0L, -50.0L},
            {-50.0L, 0.0L}
        };
    Color RED = Color(255,0,0);
    Polygon test(200.0L, 200.0L, verts, RED);
    RegularPolygon hexagon(300, 300, 8, 50, Color(0, 255, 255));
    std::vector<TwoDimensionalObject*> objects{&test, &hexagon};
    GraphicsWindow window(800, 600, objects, Color(255,255,255), Camera());
    
    window.open();
    while(window.isOpen()) {
        window.refresh();
        hexagon.rotate(0.001);
    }
}
