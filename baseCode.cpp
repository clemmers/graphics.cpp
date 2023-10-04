// g++ -o baseCode baseCode.cpp -lSDL2 -lGL -lGLU -lglut -I/usr/include/SDL2 -L/usr/lib

#include <SDL2/SDL.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <math.h>

#define PI 3.141592653589793238462643383279502884L


const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

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

void drawLine(SDL_Renderer* renderer, long double x1, long double y1, long double x2, long double y2, Color color) {
	SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer, (int) x1, (int) y1, (int) x2, (int) y2);
}

std::vector<std::vector<long double>> calculateRegularPolygonVertices (int numSides, long double circumradius) {
    std::vector<std::vector<long double>> vertices {};
        long double angle = 2 * PI / numSides;
        for (int i = 0; i < numSides; i++)
        {
            vertices.push_back(std::vector<long double>{circumradius * cos(i * angle), circumradius * sin(i * angle)});
        }
        return vertices;
}

class twoDimensionalObject {
protected:
	long double x;
	long double y;
    Color color;

public:
    twoDimensionalObject(long double xx, long double yy, Color colorr)
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

class Polygon : public twoDimensionalObject {
protected:
	std::vector<std::vector<long double>> vertices;

public:
	Polygon(long double x, long double y, std::vector<std::vector<long double>> verticess, Color color)
    : twoDimensionalObject(x, y, color), vertices(verticess) {}

    Polygon(std::vector<std::vector<long double>> verticess) : twoDimensionalObject(0.0L, 0.0L, Color()), vertices(verticess) {}

    std::vector<std::vector<long double>>* getVerticesPointer() {
        return &vertices;
    }

	void draw(SDL_Renderer* renderer) {
		int i;
		for(i = 0; i < vertices.size() - 1; i++) {
			drawLine(renderer, vertices[i][0] + x, vertices[i][1] + y, vertices[i+1][0] + x, vertices[i+1][1] + y, color);
		}
		drawLine(renderer, vertices[i][0] + x, vertices[i][1] + y, vertices[0][0] + x, vertices[0][1] + y, color);
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

void setPixel(SDL_Renderer* renderer, long double x, long double y, Color color)
{

    SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, (int) x, (int) y);
}



int main()
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("c++ graphics", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window)
    {
        std::cerr << "SDL window creation failed: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer)
    {
        std::cerr << "SDL renderer creation failed: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Event e;
    bool quit = false;
	long double x = 400.0L;
	long double y = 400.0L;
    int greenAmount = 0;
    std::vector<std::vector<long double>> verts {
        {0.0L, 0.0L},
        {0.0L, -50.0L},
        {-50.0L, -50.0L},
        {-50.0L, 0.0L}
    };
    Color RED = Color(255,0,0);
	Polygon test(200.0L, 200.0L, verts, RED);
    RegularPolygon hexagon(300, 300, 8, 50, Color(0, 0, 255));

    Uint32 prevTicks = SDL_GetTicks();
    int frameCount = 0;
    std::string windowTitle = "fps";

    while (!quit)
    {

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);
        while (SDL_PollEvent(&e) != 0)
        {
            if (e.type == SDL_QUIT)
            {
                quit = true;
            }
        }
        
        drawLine(renderer, 400.0L, 400.0L, 410.0L, 490.0L, RED);
		drawLine(renderer, 400.0L, 400.0L, 500.0L, 400.0L, Color(255,255,0));

        if(greenAmount == 255) {
            greenAmount = 0;
        } greenAmount++;
        
		
        test.getColorPointer()->setGreen(greenAmount);
        hexagon.getColorPointer()->setGreen(greenAmount);
		test.draw(renderer);
        hexagon.draw(renderer);
        if(hexagon.getX() > SCREEN_WIDTH) {
            std::cout << (0 - 2*hexagon.getCircumradius());
            hexagon.setX(0 - 2*hexagon.getCircumradius());
        } hexagon.transform(1,0);
        

		SDL_RenderPresent(renderer);

        Uint32 currentTicks = SDL_GetTicks();
        frameCount++;
        if (currentTicks - prevTicks >= 1000)
        {
            int fps = frameCount * 1000 / (currentTicks - prevTicks);
            windowTitle = "fps: " + std::to_string(fps);
            SDL_SetWindowTitle(window, windowTitle.c_str());
            
            frameCount = 0;
            prevTicks = currentTicks;
        }

    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
