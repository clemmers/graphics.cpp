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
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
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

    void setX(int x) {
        this->x = x;
    }
    
    void setY(int y) {
        this->y = y;
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
    int circumradius;

public:
    RegularPolygon(int x, int y, int numSides, int circumradius, Color color)
    : Polygon(x, y, {
        std::vector<std::vector<long double>> vertices;
        long double angle = 2 * PI / numSides;
        for (int i = 0; i < numSides; i++)
        {
            vertices.push_back(std::vector<long double>{circumradius * cos(i * angle), circumradius * sin(i * angle)});
        }
        return vertices;
    }, color) {}

};

void setPixel(SDL_Renderer* renderer, long double x, long double y, Color color)
{

    SDL_SetRenderDrawColor(renderer, color.getRed(), color.getGreen(), color.getBlue(), SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, x, y);
}

int main()
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Pixel Drawing", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
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
    while (!quit)
    {
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
		test.draw(renderer);

		SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
